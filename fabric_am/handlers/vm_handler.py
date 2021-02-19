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


class HandlerException(Exception):
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

    def _delete_vm(self):
        pass

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
                raise HandlerException(f"Missing required parameters headnode: {head_node} workernode: {worker_node} "
                                       f"flavor: {flavor}: vmname: {vmname} image: {image}")

            ansible_helper = self.create_ansible_helper(host=head_node)

            az = f"nova:{worker_node}"
            ansible_helper.add_vars(host=head_node, var_name=AmConstants.EC2_AVAILABILITY_ZONE, value=az)
            ansible_helper.add_vars(host=head_node, var_name=Constants.VM_NAME, value=vmname)
            ansible_helper.add_vars(host=head_node, var_name=Constants.FLAVOR, value=flavor)
            ansible_helper.add_vars(host=head_node, var_name=Constants.IMAGE, value=image)
            ansible_helper.add_vars(host=head_node, var_name=AmConstants.VM_PROV_OP,
                                    value=AmConstants.VM_PROV_OP_CREATE)

            pb_location = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            create_vm_pb = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_VM_CREATE]

            if create_vm_pb is None:
                raise HandlerException(f"Missing config parameters create_vm_pb: {create_vm_pb}")

            # create VM
            playbook_path = f"{pb_location}/{create_vm_pb}"
            self.logger.debug(f"Executing playbook {playbook_path}")
            ansible_helper.run_playbook(playbook_path=playbook_path)

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            self._delete_vm()
        finally:

            self.logger.info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        result = None
        try:
            self.logger.info(f"Delete invoked for unit: {unit} properties: {properties}")

            head_node = properties.get(Constants.HEAD_NODE, None)
            vmname = properties.get(Constants.VM_NAME, None)

            if head_node is None or vmname is None:
                raise HandlerException(f"Missing required parameters headnode: {head_node} vmname: {vmname}")

            ansible_helper = self.create_ansible_helper(host=head_node)

            ansible_helper.add_vars(host=head_node, var_name=Constants.VM_NAME, value=vmname)

            pb_location = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            delete_vm_pb = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_VM_DELETE]

            if pb_location is None or delete_vm_pb is None:
                raise HandlerException(f"Missing config parameters delete_vm_pb: {delete_vm_pb} "
                                       f"pb_location: {pb_location}")

            # Delete VM
            playbook_path = f"{pb_location}/{delete_vm_pb}"
            self.logger.debug(f"Executing playbook {playbook_path}")
            ansible_helper.run_playbook(playbook_path=playbook_path)

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
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

    def create_ansible_helper(self, *, host: str):
        try:
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            auth_url = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_URL]
            auth_password = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_PASSWORD]
            project_name = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_PROJECT_NAME]
            user_name = self.config[AmConstants.AUTH_SECTION][AmConstants.AUTH_USER_NAME]
            mgmt_network_name = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_MGMT_NETWORK_NAME]
            security_group = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_SECURITY_GROUP]
            key_name = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_KEY_NAME]
            public_network = self.config[AmConstants.EC2_SECTION][AmConstants.EC2_EXTERNAL_NETWORK]
            
            if inventory_path is None or auth_url is None or auth_password is None or project_name is None or \
                    user_name is None or mgmt_network_name is None or security_group is None or key_name is None or \
                    public_network is None:
                raise HandlerException(f"Missing config parameters: inventory_path: {inventory_path} "
                                       f"auth_url: {auth_url} auth_password: {auth_password} "
                                       f"project_name: {project_name} user_name: {user_name} "
                                       f"mgmt_network_name: {mgmt_network_name} security_group: {security_group} "
                                       f"key_name: {key_name} public_network: {public_network}")

            helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)

            helper.add_vars(host=host, var_name=AmConstants.AUTH_URL,value=auth_url)

            helper.add_vars(host=host, var_name=AmConstants.AUTH_PASSWORD, value=auth_password)

            helper.add_vars(host=host, var_name=AmConstants.AUTH_PROJECT_NAME, value=project_name)

            helper.add_vars(host=host, var_name=AmConstants.AUTH_USER_NAME, value=user_name)

            helper.add_vars(host=host, var_name=AmConstants.EC2_MGMT_NETWORK_NAME, value=mgmt_network_name)

            helper.add_vars(host=host, var_name=AmConstants.EC2_SECURITY_GROUP, value=security_group)

            helper.add_vars(host=host, var_name=AmConstants.EC2_KEY_NAME, value=key_name)

            helper.add_vars(host=host, var_name=AmConstants.EC2_EXTERNAL_NETWORK, value=public_network)
            return helper
        except Exception as e:
            self.logger.error(f"Error occurred while creating ansible helper: {e}")
            self.logger.error(traceback.format_exc())
            raise e
