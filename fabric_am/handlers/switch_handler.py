#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2020 FABRIC Testbed
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#
# Author: Komal Thareja (kthare10@renci.org)
import time
import traceback
from typing import Tuple

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.network_node import NodeSliver, NodeType
from jinja2 import Environment

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.utils import Utils


class SwitchHandlerException(Exception):
    """
    Switch Handler Exception
    """
    pass


class SwitchHandler(HandlerBase):
    """
    Switch Handler
    """
    test_mode = False

    @staticmethod
    def convert_to_string(unsafe_text_variable):
        if unsafe_text_variable is None:
            return unsafe_text_variable

        # Create a Jinja2 environment
        jinja_env = Environment()

        # Use the |string filter to convert AnsibleUnsafeText to string
        return jinja_env.from_string("{{ unsafe_text_variable | string }}").render(
            unsafe_text_variable=unsafe_text_variable)

    def clean_restart(self):
        self.get_logger().debug("Clean restart - begin")
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            cleanup_section = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CLEANUP]
            cleanup_playbook = f"{playbook_path}/{cleanup_section[AmConstants.CLEAN_ALL]}"
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            extra_vars = {AmConstants.OPERATION: AmConstants.OP_DELETE_ALL}
            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=cleanup_playbook,
                                  extra_vars=extra_vars, logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Failure to clean up existing VMs: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().debug("Clean restart - end")

        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CLEAN_RESTART,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        return result

    def create(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Create a VM
        :param unit: unit representing VM
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        sliver = None
        ssh_key = None
        try:
            self.get_logger().info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver) and sliver.get_type() != NodeType.Switch:
                raise SwitchHandlerException(f"Invalid Sliver instance type {type(sliver)}: "
                                             f"resource_type: {sliver.get_type()}")

            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise SwitchHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()
            ssh_key = unit_properties.get(Constants.USER_SSH_KEY, None)

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            resource_type = str(sliver.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise SwitchHandlerException(f"Missing config parameters playbook: {playbook} "
                                             f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # create switch
            extra_vars = {
                AmConstants.OPERATION: AmConstants.OP_CREATE
            }

            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=f"{playbook_path}/{playbook}",
                                  extra_vars=extra_vars, logger=self.get_logger())

            from ansible.inventory.manager import InventoryManager
            from ansible.parsing.dataloader import DataLoader
            data_loader = DataLoader()
            inventory = InventoryManager(loader=data_loader,
                                         sources=[inventory_path])
            host = inventory.get_host(hostname=f"{sliver.get_site().lower()}-p4.fabric-testbed.net")
            host_vars = host.get_vars()
            ansible_host = host_vars.get('ansible_host')
            ansible_ssh_user = host_vars.get('ansible_ssh_user')
            ansible_ssh_pass = host_vars.get('ansible_ssh_pass')

            Utils.execute_command(mgmt_ip=ansible_host, user=ansible_ssh_user, pwd=ansible_ssh_pass,
                                  logger=self.get_logger(), retry=5,
                                  command=f"echo { ansible_ssh_pass } | sudo -S reboot")

            time.sleep(1)

            Utils.verify_ssh(mgmt_ip=ansible_host, user=ansible_ssh_user, pwd=ansible_ssh_pass,
                             logger=self.get_logger(), retry=10)

            extra_vars = {
                AmConstants.OPERATION: AmConstants.OP_CONFIG,
                AmConstants.SSH_KEY: ssh_key
            }
            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=f"{playbook_path}/{playbook}",
                                  extra_vars=extra_vars, logger=self.get_logger())

            Utils.execute_command(mgmt_ip=ansible_host, user=ansible_ssh_user, pwd=ansible_ssh_pass,
                                  logger=self.get_logger(), retry=5,
                                  command=f"echo { ansible_ssh_pass } | sudo -S reboot")

            time.sleep(1)

            Utils.verify_ssh(mgmt_ip=ansible_host, user=ansible_ssh_user, pwd=ansible_ssh_pass,
                             logger=self.get_logger(), retry=10)

            extra_vars = {
                AmConstants.OPERATION: AmConstants.OP_POST_REBOOT,
            }
            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=f"{playbook_path}/{playbook}",
                                  extra_vars=extra_vars, logger=self.get_logger())

            Utils.verify_ssh(mgmt_ip=ansible_host, user=ansible_ssh_user, pwd=ansible_ssh_pass,
                             logger=self.get_logger(), retry=10)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            self.__cleanup(sliver=sliver, ssh_key=ssh_key)
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:

            self.get_logger().info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Delete a provisioned VM
        :param unit: unit representing VM
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Delete invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if sliver is None:
                raise SwitchHandlerException(f"Unit # {unit} has no assigned slivers")

            if not isinstance(sliver, NodeSliver) and sliver.get_type() != NodeType.Switch:
                raise SwitchHandlerException(f"Invalid Sliver instance type {type(sliver)}: "
                                             f"resource_type: {sliver.get_type()}")

            unit_properties = unit.get_properties()
            ssh_key = unit_properties.get(Constants.USER_SSH_KEY, None)

            self.__cleanup(sliver=sliver, ssh_key=ssh_key)
        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
        finally:

            self.get_logger().info(f"Delete completed")
        return result, unit

    def modify(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Modify a provisioned Unit
        :param unit: unit representing Switch
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Modify invoked for unit: {unit}")
            self.get_logger().info(f"Modify nothing to do!")
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:

            self.get_logger().info(f"Modify completed")
        return result, unit

    def poa(self, unit: ConfigToken, data: dict) -> Tuple[dict, ConfigToken]:
        """
        POA a provisioned Unit
        :param unit: unit representing Switch
        :param data: data
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"POA invoked for unit: {unit}")
            self.get_logger().info(f"POA nothing to do!")
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:

            self.get_logger().info(f"POA completed")
        return result, unit

    def __cleanup(self, *, sliver: NodeSliver, ssh_key: str, raise_exception: bool = False):
        """
        Cleanup VM and detach PCI devices
        :param sliver: Sliver
        :param ssh_key: ssh_key
        :param raise_exception: Raise exception if raise_exception flag is True
        :return:
        """
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            resource_type = str(sliver.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]

            # reset switch
            extra_vars = {
                AmConstants.OPERATION: AmConstants.OP_DELETE,
                AmConstants.SSH_KEY: ssh_key
            }
            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=f"{playbook_path}/{playbook}",
                                  extra_vars=extra_vars, logger=self.get_logger())

        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup error: {e}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e

