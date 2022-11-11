#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2022 FABRIC Testbed
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
from typing import Tuple

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.capacities_labels import Labels, Capacities
from fim.slivers.network_service import NetworkServiceSliver, MirrorDirection, NSLayer

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper
from networkx.generators.tests.test_small import null
    
class OessHandlerException(Exception):
    """
    OESS Handler Exception
    """
    pass


class OessHandler(HandlerBase):
    """
    OESS Handler
    """

    def get_ansible_python_interpreter(self) -> str:
        return self.get_config()[AmConstants.ANSIBLE_SECTION][
            AmConstants.ANSIBLE_PYTHON_INTERPRETER]

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
        unit_id = None
        try:
            # self.get_logger().info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise OessHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()  # use: TBD

            resource_type = str(sliver.get_type())

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise OessHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"

            if sliver.get_labels() is None or sliver.get_labels().local_name is None:
                # truncate service_name length to no greater than 53 (36+1+16)
                sliver_name = sliver.get_name()[:16] if len(sliver.get_name()) > 16 else sliver.get_name()
                service_name = f'{sliver_name}-{unit_id}'
            else:
                service_name = sliver.get_labels().local_name
            service_type = resource_type.lower()
            if service_type == 'l2ptp':
                service_data = self.__l2ptp_create_data(sliver, service_name)
            elif service_type == 'l3vpn':
                service_data = self.__l3vpn_create_data(sliver, service_name)
            else:
                raise OessHandlerException(f'unrecognized network service type "{service_type}"')

            extra_vars = service_data
            print(json.dumps(extra_vars))
            
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger(),
                                           ansible_python_interpreter=self.get_ansible_python_interpreter())
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.get_logger().debug(f"Executing playbook {playbook_path_full} to create Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)
            ansible_callback = ansible_helper.get_result_callback()
            unreachable = ansible_callback.get_json_result_unreachable()
            if unreachable:
                raise OessHandlerException(f'network service {service_name} was not committed due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.get_logger())
                raise OessHandlerException(f'network service {service_name} was not committed due to config error')
            ok = ansible_callback.get_json_result_ok()
            if ok:
                if not ok['changed']:
                    self.get_logger().info(f'network service {service_name} was committed ok but without change')

        except Exception as e:
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver, unit_id=unit_id)
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Delete invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if sliver is None:
                raise OessHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_id = str(unit.get_reservation_id())
            self.__cleanup(sliver=sliver, raise_exception=True, unit_id=unit_id)
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
        raise NetHandlerException(f"NetworkServiceSliver modify action is not supported yet...")

    def __cleanup(self, *, sliver: NetworkServiceSliver, unit_id: str, raise_exception: bool = False):
        if sliver.get_labels() is None or sliver.get_labels().local_name is None:
            # truncate service_name length to no greater than 53 (36+1+16)
            sliver_name = sliver.get_name()[:16] if len(sliver.get_name()) > 16 else sliver.get_name()
            service_name = f'{sliver_name}-{unit_id}'
        else:
            service_name = sliver.get_labels().local_name
        resource_type = str(sliver.get_type())
        service_type = resource_type.lower()
        if service_type == 'l2ptp':
            service_data = self.__l2ptp_delete_data(sliver, service_name)
        elif service_type == 'l3vpn':
            service_data = self.__l3vpn_delete_data(sliver, service_name)
        elif service_type == 'l2cloud':
            service_data = self.__l2cloud_delete_data(sliver, service_name)
        elif service_type == 'l3cloud':
            service_data = self.__l3cloud_delete_data(sliver, service_name)
            
        extra_vars = service_data
        
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise NetHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger(),
                                           ansible_python_interpreter=self.get_ansible_python_interpreter())
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.get_logger().debug(f"Executing playbook {playbook_path_full} to delete Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)
            ansible_callback = ansible_helper.get_result_callback()
            unreachable = ansible_callback.get_json_result_unreachable()
            if unreachable:
                raise NetHandlerException(f'network service {service_name} was not cleaned up due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.get_logger())
                raise NetHandlerException(f'network service {service_name} was not cleaned up due to config error')
            ok = ansible_callback.get_json_result_ok()
            if ok:
                if not ok['changed']:
                    self.get_logger().info(f'network service {service_name} was cleaned up ok but without change')

        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup {unit_id} error: {e}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e

    def __l2ptp_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        if len(sliver.interface_info.interfaces) != 2:
            raise OessHandlerException(
                f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise OessHandlerException(f'l2ptp - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise NetHandlerException(f'l2ptp - interface "{interface_name}" has no "local_name" label')
            endpoint['bandwidth'] = caps.bw
            endpoint['interface'] = labs.local_name
            endpoint['tag'] = labs.vlan
            if labs.account_id != None:
                endpoint['cloud_account_id'] = labs.account_id
            endpoint_list.append(endpoint)

        data = {"description": service_name, 
                "op": "create",
                "level": "L2",
                "l2_endpoints": endpoint_list}
        
        return data

    def __l2ptp_delete_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        if len(sliver.interface_info.interfaces) != 2:
            raise OessHandlerException(
                f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L2",
                "description": service_name}
        
        return data

    def __l3vpn_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        local_asn = ''
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise OessHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise OessHandlerException(f'l3vpn - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise OessHandlerException(f'l3vpn - interface "{interface_name}" has no "local_name" label')
            if local_asn and  local_asn != labs.asn:
                self.get_logger().error(f"local asn is inconsistant in __l3cloud_create_data")
                raise OessHandlerException(f'l3vpn - interface "{interface_name}" has inconsistant local_asn')
            elif not local_asn:
                local_asn = labs.asn;
                
            endpoint['bandwidth'] = caps.bw
            endpoint['interface'] = labs.local_name
            endpoint['tag'] = labs.vlan
            endpoint['jumbo'] = caps.jumbo
            if labs.account_id != None:
                endpoint['cloud_account_id'] = labs.account_id 
            endpoint['peers'] = {}
            if interface_name in sliver.get_peer_labels():
                endpoint['peers']  =  [sliver.get_peer_labels()[interface_name]]
            else:
                self.get_logger().error(f"Peers not found in __l3vpn_create_data")
                raise OessHandlerException(f'l3vpn - interface "{interface_name}" has no peers')
                    
            endpoint_list.append(endpoint)

        data = {"name": service_name,
                "description": service_name,
                "op": "create",
                "level": "L3",
                "asn": local_asn,
                "l3_endpoints": endpoint_list}
        
        return data

    def __l3vpn_delete_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise OessHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L3",
                "name": service_name}
        
        return data

    def __l2cloud_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise OessHandlerException(
        #         f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise OessHandlerException(f'l2ptp - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise OessHandlerException(f'l2ptp - interface "{interface_name}" has no "local_name" label')
            endpoint['bandwidth'] = caps.bw
            endpoint['interface'] = labs.local_name
            endpoint['tag'] = labs.vlan
            endpoint['cloud_account_id'] = labs.account_id
            endpoint_list.append(endpoint)

        data = {"description": service_name, 
                "op": "create",
                "level": "L2",
                "l2_endpoints": endpoint_list}
        
        return data

    def __l2cloud_delete_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise OessHandlerException(
        #         f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L2",
                "description": service_name}
        
        return data

    def __l3cloud_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        local_asn = ''
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise OessHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise OessHandlerException(f'l3cloud - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise OessHandlerException(f'l3cloud - interface "{interface_name}" has no "local_name" label')
            if local_asn and  local_asn != labs.asn:
                self.get_logger().error(f"local asn is inconsistant in __l3cloud_create_data")
                raise OessHandlerException(f'l3cloud - interface "{interface_name}" has inconsistant local_asn')
            elif not local_asn:
                local_asn = labs.asn;
                
            endpoint['bandwidth'] = caps.bw
            endpoint['interface'] = labs.local_name
            endpoint['tag'] = labs.vlan
            endpoint['jumbo'] = caps.jumbo
            endpoint['cloud_account_id'] = labs.account_id
            endpoint['peers'] = {}
            if interface_name in sliver.get_peer_labels():
                endpoint['peers']  =  [sliver.get_peer_labels()[interface_name]]
            else:
                self.get_logger().error(f"Peers not found in __l3cloud_create_data")
                raise OessHandlerException(f'l3cloud - interface "{interface_name}" has no peers')
                    
            endpoint_list.append(endpoint)

        data = {"name": service_name,
                "description": service_name,
                "op": "create",
                "level": "L3",
                "asn": local_asn,
                "l3_endpoints": endpoint_list}
        
        return data

    def __l3cloud_delete_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise OessHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L3",
                "name": service_name}
        
        return data
    
    def clean_restart(self):
        return;
