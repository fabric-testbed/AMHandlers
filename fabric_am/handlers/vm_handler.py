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
import traceback
from typing import Tuple

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class VmHandlerException(Exception):
    pass


class VMHandler(HandlerBase):
    def __init__(self, logger, properties: dict):
        self.logger = logger
        self.properties = properties
        self.config = None

        config_properties_file = self.properties.get(AmConstants.CONFIG_PROPERTIES_FILE, None)
        if config_properties_file is None:
            return

        self.config = self.load_config(path=config_properties_file)

    def create(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        """
        Create a VM
        :param unit: unit representing VM
        :param properties: properties
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        playbook_path = None
        inventory_path = None
        head_node = None
        vmname = None

        try:
            self.logger.info(f"Create invoked for unit: {unit} properties: {properties}")

            head_node = properties.get(Constants.HEAD_NODE, None)
            worker_node = properties.get(Constants.WORKER_NODE, None)
            flavor = properties.get(Constants.FLAVOR, None)
            vmname = properties.get(Constants.VM_NAME, None)
            image = properties.get(Constants.IMAGE, None)
            az = f"nova:{worker_node}"

            if head_node is None or worker_node is None or flavor is None or vmname is None or image is None:
                raise VmHandlerException(f"Missing required parameters headnode: {head_node} workernode: {worker_node} "
                                         f"flavor: {flavor}: vmname: {vmname} image: {image}")

            pb_location = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            pb_vm_prov = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_VM_PROVISIONING]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if pb_vm_prov is None or inventory_path is None:
                raise VmHandlerException(f"Missing config parameters pb_vm_prov: {pb_vm_prov} "
                                         f"pb_location: {pb_location} inventory_path: {inventory_path}")

            # create VM
            playbook_path = f"{pb_location}/{pb_vm_prov}"
            instance_props = self.__create_vm(playbook_path=playbook_path, inventory_path=inventory_path, host=head_node,
                                              vm_name=vmname, image=image, flavor=flavor, avail_zone=az)

            # Attach FIP
            fip_props = self.__attach_fip(playbook_path=playbook_path, inventory_path=inventory_path, host=head_node,
                                          vm_name=vmname)

            for x, y in fip_props.items():
                instance_props[x] = y

        except Exception as e:
            # Delete VM in case of failure
            if playbook_path is not None and inventory_path is not None and head_node is not None \
                    and vmname is not None:
                self.__delete_vm(playbook_path=playbook_path, inventory_path=inventory_path, host=head_node,
                                 vm_name=vmname)

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        """
        Delete a provisioned VM
        :param unit: unit representing VM
        :param properties:  properties
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.logger.info(f"Delete invoked for unit: {unit} properties: {properties}")

            head_node = properties.get(Constants.HEAD_NODE, None)
            vmname = properties.get(Constants.VM_NAME, None)

            if head_node is None or vmname is None:
                raise VmHandlerException(f"Missing required parameters headnode: {head_node} vmname: {vmname}")

            pb_location = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            pb_vm_prov = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_VM_PROVISIONING]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if pb_vm_prov is None or inventory_path is None:
                raise VmHandlerException(f"Missing config parameters pb_vm_prov: {pb_vm_prov} "
                                         f"pb_location: {pb_location} inventory_path: {inventory_path}")

            # Delete VM
            playbook_path = f"{pb_location}/{pb_vm_prov}"
            props = self.__delete_vm(playbook_path=playbook_path, inventory_path=inventory_path, host=head_node,
                                     vm_name=vmname)

        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Delete completed")
        return result, unit

    def modify(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        """
        Modify a provisioned Unit
        :param unit: unit representing VM
        :param properties: properties
        :return: tuple of result status and the unit
        """
        result = None
        try:
            self.logger.info(f"Modify invoked for unit: {unit} properties: {properties}")

        except Exception as e:
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        finally:
            self.logger.info(f"Modify completed")
        return result, unit

    def __create_vm(self, *, playbook_path: str, inventory_path: str, host: str, vm_name: str,
                    avail_zone: str, image: str, flavor: str) -> dict:
        """
        Invoke ansible playbook to provision a VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param vm_name: VM Name
        :param avail_zone: Availability Zone
        :param image: Image
        :param flavor: Flavor
        :return: dictionary containing created instance details
        """
        ansible_helper = None
        try:
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)

            ansible_helper.add_vars(host=host, var_name=AmConstants.VM_PROV_OP, value=AmConstants.VM_PROV_OP_CREATE)
            ansible_helper.add_vars(host=host, var_name=AmConstants.EC2_AVAILABILITY_ZONE, value=avail_zone)
            ansible_helper.add_vars(host=host, var_name=Constants.VM_NAME, value=vm_name)
            ansible_helper.add_vars(host=host, var_name=Constants.FLAVOR, value=flavor)
            ansible_helper.add_vars(host=host, var_name=Constants.IMAGE, value=image)

            self.logger.debug(f"Executing playbook {playbook_path} to create VM")
            ansible_helper.run_playbook(playbook_path=playbook_path)
            ok = ansible_helper.get_result_callback().get_json_result_ok(host=host)

            result = {
                AmConstants.SERVER_VM_STATE:ok[AmConstants.SERVER][AmConstants.SERVER_VM_STATE],
                AmConstants.SERVER_INSTANCE_NAME: ok[AmConstants.SERVER][AmConstants.SERVER_INSTANCE_NAME],
                AmConstants.SERVER_ACCESS_IPv4: ok[AmConstants.SERVER][AmConstants.SERVER_ACCESS_IPv4]
            }
            self.logger.debug(f"Returning properties {result}")

            return result
        finally:
            if ansible_helper is not None:
                self.logger.debug(f"OK: {ansible_helper.get_result_callback().get_json_result_ok(host=host)}")
                self.logger.error(f"Failed: {ansible_helper.get_result_callback().get_json_result_failed(host=host)}")
                self.logger.error(f"Unreachable: "
                                  f"{ansible_helper.get_result_callback().get_json_result_unreachable(host=host)}")

    def __delete_vm(self, *, playbook_path: str, inventory_path: str, host: str, vm_name: str) -> bool:
        """
        Invoke ansible playbook to remove a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param vm_name: VM Name
        :return: True or False representing success/failure
        """
        ansible_helper = None
        try:
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)

            ansible_helper.add_vars(host=host, var_name=AmConstants.VM_PROV_OP, value=AmConstants.VM_PROV_OP_DELETE)
            ansible_helper.add_vars(host=host, var_name=Constants.VM_NAME, value=vm_name)

            self.logger.debug(f"Executing playbook {playbook_path} to delete VM")
            ansible_helper.run_playbook(playbook_path=playbook_path)
            return True
        finally:
            if ansible_helper is not None:
                self.logger.debug(f"OK: {ansible_helper.get_result_callback().get_json_result_ok(host=host)}")
                self.logger.error(f"Failed: {ansible_helper.get_result_callback().get_json_result_failed(host=host)}")
                self.logger.error(f"Unreachable: "
                                  f"{ansible_helper.get_result_callback().get_json_result_unreachable(host=host)}")

    def __attach_fip(self, *, playbook_path: str, inventory_path: str, host: str, vm_name: str) -> dict:
        """
        Invoke ansible playbook to attach a floating IP to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param vm_name: VM Name
        :return: dictionary containing created floating ip details
        """
        ansible_helper = None
        try:
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
            ansible_helper.add_vars(host=host, var_name=AmConstants.VM_PROV_OP, value=AmConstants.VM_PROV_OP_ATTACH_FIP)
            ansible_helper.add_vars(host=host, var_name=Constants.VM_NAME, value=vm_name)

            self.logger.debug(f"Executing playbook {playbook_path} to attach FIP")
            ansible_helper.run_playbook(playbook_path=playbook_path)

            ok = ansible_helper.get_result_callback().get_json_result_ok(host=host)

            result = {AmConstants.FLOATING_IP: ok[AmConstants.FLOATING_IP][AmConstants.FLOATING_IP_ADDRESS],
                      AmConstants.FLOATING_IP_MAC_ADDRESS: ok[AmConstants.FLOATING_IP][AmConstants.FLOATING_IP_PROPERTIES][AmConstants.FLOATING_IP_PORT_DETAILS][AmConstants.FLOATING_IP_MAC_ADDRESS]}
            self.logger.debug(f"Returning properties {result}")
            return result
        finally:
            if ansible_helper is not None:
                self.logger.debug(f"OK: {ansible_helper.get_result_callback().get_json_result_ok(host=host)}")
                self.logger.error(f"Failed: {ansible_helper.get_result_callback().get_json_result_failed(host=host)}")
                self.logger.error(f"Unreachable: "
                                  f"{ansible_helper.get_result_callback().get_json_result_unreachable(host=host)}")

