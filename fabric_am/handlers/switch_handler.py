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

import paramiko
from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.network_node import NodeSliver, NodeType
from jinja2 import Environment

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


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
            self.__execute_ansible(inventory_path=inventory_path, playbook_path=cleanup_playbook,
                                   extra_vars=extra_vars)
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

        unit_id = None
        sliver = None
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
                AmConstants.OPERATION: AmConstants.OP_CREATE,
                AmConstants.SSH_KEY: ssh_key
            }
            self.__execute_ansible(inventory_path=inventory_path, playbook_path=f"{playbook_path}/{playbook}",
                                   extra_vars=extra_vars)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                time.sleep(5)
                self.__cleanup(sliver=sliver, unit_id=unit_id)

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

            unit_id = str(unit.get_reservation_id())
            unit_properties = unit.get_properties()
            project_id = unit_properties.get(Constants.PROJECT_ID, None)
            self.__cleanup(sliver=sliver, unit_id=unit_id, project_id=project_id)
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

    def __cleanup(self, *, sliver: NodeSliver, unit_id: str, raise_exception: bool = False):
        """
        Cleanup VM and detach PCI devices
        :param sliver: Sliver
        :param unit_id: Unit Id
        :param project_id: Project Id
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
                AmConstants.OPERATION: AmConstants.OP_DELETE
            }
            self.__execute_ansible(inventory_path=inventory_path, playbook_path=f"{playbook_path}/{playbook}",
                                   extra_vars=extra_vars)

        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup {unit_id} error: {e}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e

    def __execute_command(self, *, mgmt_ip: str, user: str, command: str, timeout: int = 60, retry: int = 3):
        """
        Execute a command on the VM
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param command Command to execute
        :param timeout Timeout in seconds
        :param retry Number of retries
        :return:
        """
        for i in range(retry):
            try:
                # Construct the SSH client
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                key_file = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]
                pkey = paramiko.RSAKey.from_private_key_file(key_file)
                ssh.connect(mgmt_ip, username=user, timeout=timeout, pkey=pkey)

                # Execute the command
                stdin, stdout, stderr = ssh.exec_command(command)
                output = stdout.readlines()
                ssh.close()
                return output
            except Exception as e:
                self.get_logger().error(f"Exception : {e}")
                self.get_logger().error(traceback.format_exc())
                if i < retry - 1:
                    time.sleep(timeout)
                    self.get_logger().info(f"Retrying command {command} on VM {mgmt_ip}")
                else:
                    self.get_logger().error(f"Failed to execute command {command} on VM {mgmt_ip}")
                    raise e

    def __verify_ssh(self, *, mgmt_ip: str, user: str, timeout: int = 60, retry: int = 10):
        """
        Verify that the VM is accessible via SSH
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param retry Number of retries
        :param retry_interval Timeout in seconds

        """
        command = f"echo test ssh from {mgmt_ip} > /tmp/fabric_execute_script.sh; " \
                  f"chmod +x /tmp/fabric_execute_script.sh; /tmp/fabric_execute_script.sh"

        try:
            output = self.__execute_command(mgmt_ip=mgmt_ip, user=user, command=command,
                                            timeout=timeout, retry=retry)
            self.get_logger().info(f"Output: {output}")
        except Exception as e:
            pass

    def __execute_ansible(self, *, inventory_path: str, playbook_path: str, extra_vars: dict, sources: str = None,
                          private_key_file: str = None, host_vars: dict = None, host: str = None, user: str = None):
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger(),
                                       sources=sources)

        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        if host is not None and host_vars is not None and len(host_vars) > 0:
            for key, value in host_vars.items():
                ansible_helper.add_vars(host=host, var_name=key, value=value)

        self.get_logger().info(f"Executing playbook {playbook_path} extra_vars: {extra_vars} host_vars: {host_vars}")
        ansible_helper.run_playbook(playbook_path=playbook_path, private_key_file=private_key_file, user=user)
        return ansible_helper.get_result_callback().get_json_result_ok()
