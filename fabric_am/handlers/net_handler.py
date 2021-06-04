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
        sliver = None

        try:
            self.logger.info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties() # use: TBD

            resource_type = str(sliver.get_type())

            assert resource_type == 'Network'
            playbook_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"

            service_name = f'{unit_id}-{siver.get_name()}'
            service_type = sliver.get_service_type()

            # TODO: get NSO service params from sliver and assemble into `service_data`
            service_data = {}
            data = {
                "tailf-ncs:services": {
                    f'{service_type}:{service_type}': [service_data]
                }
            }

            extra_vars = {
                "service_name": service_name,
                "data": data
            }

            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.logger.debug(f"Executing playbook {playbook_path_full} to create Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)

            # TODO: handle NSO result and errors
            ok = ansible_helper.get_result_callback().get_json_result_ok()
            # debugging
            ansible_helper.get_result_callback().dump_all_ok(logger=self.logger)
            ansible_helper.get_result_callback().dump_all_failed(logger=self.logger)
            ansible_helper.get_result_callback().dump_all_unreachable(logger=self.logger)

        except Exception as e:
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver, unit_id=unit_id)
                unit.sliver.label_allocations.instance = None

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Create completed")
        return result, unit

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
