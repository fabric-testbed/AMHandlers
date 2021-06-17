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
from fim.slivers.capacities_labels import Labels, Capacities
from fim.slivers.network_service import NetworkServiceSliver

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
                raise NetHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()  # use: TBD

            resource_type = str(sliver.get_type())

            playbook_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise NetHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"

            if sliver.get_labels() is None or sliver.get_labels().local_name is None:
                service_name = f'{unit_id}-{sliver.get_name()}'
            else:
                service_name = sliver.get_labels().local_name
            service_type = resource_type.lower()
            if service_type == 'l2bridge':
                service_data = self.__l2bridge_create_data(sliver, service_name)
            else:
                raise NetHandlerException(f'unrecognized network service type "{service_type}"')
            data = {
                "tailf-ncs:services": {
                    f'{service_type}:{service_type}': [service_data]
                }
            }
            extra_vars = {
                "service_name": service_name,
                "service_type": service_type,
                "service_action": "create",
                "data": data
            }

            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.logger.debug(f"Executing playbook {playbook_path_full} to create Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)
            ansible_callback = ansible_helper.get_result_callback()
            unreachable = ansible_callback.get_json_result_unreachable()
            if unreachable:
                raise NetHandlerException(f'network service {service_name} was not committed due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.logger)
                raise NetHandlerException(f'network service {service_name} was not committed due to config error')
            ok = ansible_callback.get_json_result_ok()
            if ok:
                if not ok['changed']:
                    self.logger.info(f'network service {service_name} was committed ok but without change')

        except Exception as e:
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver, unit_id=unit_id)
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:
            self.logger.info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.logger.info(f"Delete invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if sliver is None:
                raise NetHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_id = str(unit.get_reservation_id())
            self.__cleanup(sliver=sliver, raise_exception=True, unit_id=unit_id)
        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:
            self.logger.info(f"Delete completed")
        return result, unit

    def modify(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        raise NetHandlerException(f"NetworkServiceSliver modify action is not supported yet...")

    def __cleanup(self, *, sliver: NetworkServiceSliver, unit_id: str, raise_exception: bool = False):
        if sliver.get_labels() is None or sliver.get_labels().local_name is None:
            service_name = f'{unit_id}-{sliver.get_name()}'
        else:
            service_name = sliver.get_labels().local_name
        resource_type = str(sliver.get_type())
        service_type = resource_type.lower()
        data = {
            "tailf-ncs:services": {
                f'{service_type}:{service_type}': [ {
                        "name": f'{service_name}',
                        "__state": "absent"
                    }]
            }
        }
        extra_vars = {
            "service_name": service_name,
            "service_type": service_type,
            "service_action": "delete",
            "data": data
        }
        try:
            playbook_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise NetHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.logger.debug(f"Executing playbook {playbook_path_full} to delete Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)
            ansible_callback = ansible_helper.get_result_callback()
            unreachable = ansible_callback.get_json_result_unreachable()
            if unreachable:
                raise NetHandlerException(f'network service {service_name} was not cleaned up due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.logger)
                raise NetHandlerException(f'network service {service_name} was not cleaned up due to config error')
            ok = ansible_callback.get_json_result_ok()
            if ok:
                if not ok['changed']:
                    self.logger.info(f'network service {service_name} was cleaned up ok but without change')

        except Exception as e:
            self.logger.error(f"Exception occurred in cleanup {e}")
            self.logger.error(traceback.format_exc())
            if raise_exception:
                raise e

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

    @staticmethod
    def __l2bridge_create_data(sliver: NetworkServiceSliver, service_name: str) -> dict:
        device_name = None
        interfaces = []
        data = {"name": service_name, "interface": interfaces}
        for interface_name in sliver.interface_info.interfaces:
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise NetHandlerException(f'l2bridge - interface "{interface_name}" has no "device_name" label')
            if device_name is None:
                device_name = labs.device_name
                data['device'] = device_name
            elif device_name != labs.device_name:
                raise NetHandlerException(
                    f'l2bridge - has two different device_name "{device_name}" and "{labs.device_name}"')
            interface = {}
            if labs.local_name is None:
                raise NetHandlerException(f'l2bridge - interface "{interface_name}" has no "local_name" label')
            interface_type_id = re.findall(r'(\w+)(\d.+)', labs.local_name)
            if not interface_type_id or len(interface_type_id[0]) != 2:
                raise NetHandlerException(f'l2bridge - interface "{interface_name}" has malformed "local_name" label')
            interface['type'] = interface_type_id[0][0]
            interface['id'] = interface_type_id[0][1]
            if labs.vlan is not None:
                interface['outervlan'] = labs.vlan
                """
                if labs.inner_vlan is not None:
                    interface['innervlan'] = labs.inner_vlan
                """
            if caps.bw is not None and caps.bw != 0:
                interface['bandwidth'] = caps.bw
                if caps.burst_size is not None and caps.burst_size != 0:
                    interface['burst-size'] = caps.burst_size
            interfaces.append(interface)
        if not interfaces:
            raise NetHandlerException(f'l2bridge - none valid interface is defined in sliver')
        return data

