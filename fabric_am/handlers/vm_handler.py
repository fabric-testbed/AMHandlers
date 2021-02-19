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
        self._load()

    def _load(self):
        config_properties_file = self.properties.get(AmConstants.CONFIG_PROPERTIES_FILE, None)
        if config_properties_file is None:
            return

        self.config = self.load_config(path=config_properties_file)

    def get_config(self) -> dict:
        return self.config

    def create(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        result = None
        try:
            self.logger.info(f"Create invoked for unit: {unit} properties: {properties}")
            self.logger.info(f"Config: {self.get_config()}")

            head_node = properties.get(Constants.HEAD_NODE, None)
            worker_node = properties.get(Constants.WORKER_NODE, None)
            flavor = properties.get(Constants.FLAVOR, None)
            vmname = properties.get(Constants.VM_NAME, None)
            image = properties.get(Constants.IMAGE, None)

            if head_node is None or worker_node is None or flavor is None or vmname is None or image is None:
                raise VmHandlerException(f"Missing required parameters headnode: {head_node} workernode: {worker_node} "
                                         f"flavor: {flavor}: vmname: {vmname} image: {image}")

            pb_location = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            pb_vm_prov = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_VM_PROVISIONING]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if pb_vm_prov is None or inventory_path is None:
                raise VmHandlerException(f"Missing config parameters pb_vm_prov: {pb_vm_prov} "
                                         f"pb_location: {pb_location} inventory_path: {inventory_path}")

            auth_params = self.get_auth_params()

            # create VM
            az = f"nova:{worker_node}"
            playbook_path = f"{pb_location}/{pb_vm_prov}"
            props = self.create_vm(playbook_path=playbook_path, inventory_path=inventory_path, host=head_node,
                                   auth_vars=auth_params, vm_name=vmname, image=image, flavor=flavor, avail_zone=az)

            # Attach FIP
            props2 = self.attach_fip(playbook_path=playbook_path, inventory_path=inventory_path, host=head_node,
                                     auth_vars=auth_params, vm_name=vmname)

        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        result = None
        try:
            self.logger.info(f"Delete invoked for unit: {unit} properties: {properties}")

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

    def get_auth_params(self) -> dict:
        try:
            result = {}

            result[AmConstants.AUTH_URL] = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_URL]

            result[AmConstants.AUTH_PASSWORD] = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_PASSWORD]

            result[AmConstants.AUTH_PROJECT_NAME] = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_PROJECT_NAME]

            result[AmConstants.AUTH_USER_NAME] = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_USER_NAME]

            result[AmConstants.EC2_MGMT_NETWORK_NAME] = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_MGMT_NETWORK_NAME]

            result[AmConstants.EC2_SECURITY_GROUP] = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_SECURITY_GROUP]

            result[AmConstants.EC2_KEY_NAME] = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_KEY_NAME]

            result[AmConstants.EC2_EXTERNAL_NETWORK] = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_EXTERNAL_NETWORK]

            return result
        except Exception as e:
            self.logger.error(f"Error occurred while grabbing Auth parameters: {e}")
            self.logger.error(traceback.format_exc())
            raise e

    def create_vm(self, *, playbook_path: str, inventory_path: str, auth_vars: dict, host: str,
                  vm_name: str, avail_zone: str, image: str, flavor: str) -> dict:
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        for key, value in auth_vars.items():
            ansible_helper.add_vars(host=host, var_name=key, value=value)
        ansible_helper.add_vars(host=host, var_name=AmConstants.VM_PROV_OP, value=AmConstants.VM_PROV_OP_CREATE)

        ansible_helper.add_vars(host=host, var_name=AmConstants.EC2_AVAILABILITY_ZONE, value=avail_zone)
        ansible_helper.add_vars(host=host, var_name=Constants.VM_NAME, value=vm_name)
        ansible_helper.add_vars(host=host, var_name=Constants.FLAVOR, value=flavor)
        ansible_helper.add_vars(host=host, var_name=Constants.IMAGE, value=image)

        self.logger.debug(f"Executing playbook {playbook_path}")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        # TODO process properties and return a dict
        return {}

    def delete_vm(self, *, playbook_path: str, inventory_path: str, auth_vars: dict, host: str, vm_name: str) -> bool:
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        for key, value in auth_vars.items():
            ansible_helper.add_vars(host=host, var_name=key, value=value)
        ansible_helper.add_vars(host=host, var_name=AmConstants.VM_PROV_OP, value=AmConstants.VM_PROV_OP_DELETE)
        ansible_helper.add_vars(host=host, var_name=Constants.VM_NAME, value=vm_name)

        self.logger.debug(f"Executing playbook {playbook_path}")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        return True

    def attach_fip(self, *, playbook_path: str, inventory_path: str, auth_vars: dict, host: str, vm_name: str) -> dict:
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        for key, value in auth_vars.items():
            ansible_helper.add_vars(host=host, var_name=key, value=value)
        ansible_helper.add_vars(host=host, var_name=AmConstants.VM_PROV_OP, value=AmConstants.VM_PROV_OP_ATTACH_FIP)
        ansible_helper.add_vars(host=host, var_name=Constants.VM_NAME, value=vm_name)

        self.logger.debug(f"Executing playbook {playbook_path}")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        # TODO process properties and return a dict
        return {}
