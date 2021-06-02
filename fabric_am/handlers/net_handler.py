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
# Author: 
import json
import re
import traceback
from typing import Tuple, List

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.network_node import NodeSliver

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class NetHandlerException(Exception):
    """
    VM Handler Exception
    """
    pass


class NetHandler(HandlerBase):
    """
    Network Handler
    """
    def __init__(self, logger, properties: dict):
        self.logger = logger
        self.properties = properties
        self.config = None

        config_properties_file = self.properties.get(AmConstants.CONFIG_PROPERTIES_FILE, None)
        if config_properties_file is None:
            return

        self.config = self.load_config(path=config_properties_file)

    def create(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Create a Network Service
        :param unit: unit representing an NSO Network Service
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        unit_id = None

        inventory_path = "/home/ezra/repos/NetworkController/nso-services/ansible/inventory"
        playbook_path = "/home/ezra/repos/NetworkController/nso-services/ansible/l2bridge-data.yaml"
        
        self.logger.info(f"Create invoked for unit: {unit}")
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)

        service_name = "fabric-l2bridge-test-t1"
        
        data = {
            "tailf-ncs:services": {
                "l2bridge:l2bridge": [
                    {
                        "device": "renci-ncs55-0",
                        "interface": [
                            {
                                "id": "0/0/0/13",
                                "outervlan": "100",
                                "type": "HundredGigE"
                            },
                            {
                                "id": "0/0/0/25",
                                "outervlan": "100",
                                "type": "HundredGigE"
                            }
                        ],
                        "name": service_name
                    }
                ]
            }
        }
        
        extra_vars = {
            "service_name": service_name,
            "data": data
        }
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.logger.debug(f"Executing playbook {playbook_path} to create Network Service")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        ok = ansible_helper.get_result_callback().get_json_result_ok()

        result = {

        }
        self.logger.debug(f"Returning properties {result}")

        return result

    def delete():
        pass

    def modify():
        pass
    
    @staticmethod
    def __get_default_user(image: str) -> str:
        """
        Return default SSH user name
        :return default ssh user name
        """
        if AmConstants.CENTOS_DEFAULT_USER in image:
            return AmConstants.CENTOS_DEFAULT_USER
        elif AmConstants.UBUNTU_DEFAULT_USER in image:
            return AmConstants.UBUNTU_DEFAULT_USER
        else:
            return AmConstants.ROOT_USER
