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
from typing import Tuple

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
            self.get_logger().info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise NetHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()  # use: TBD

            resource_type = str(sliver.get_type())

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise NetHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"

            if sliver.get_labels() is None or sliver.get_labels().local_name is None:
                # truncate service_name length to no greater than 53 (36+1+16)
                sliver_name = sliver.get_name()[:16] if len(sliver.get_name()) > 16 else sliver.get_name()
                service_name = f'{sliver_name}-{unit_id}'
            else:
                service_name = sliver.get_labels().local_name
            service_type = resource_type.lower()
            if service_type == 'l2bridge':
                service_data = self.__l2bridge_create_data(sliver, service_name)
            elif service_type == 'l2ptp':
                service_data = self.__l2ptp_create_data(sliver, service_name)
            elif service_type == 'l2sts':
                service_data = self.__l2sts_create_data(sliver, service_name)
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
            print(json.dumps(extra_vars))
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.get_logger().debug(f"Executing playbook {playbook_path_full} to create Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)
            ansible_callback = ansible_helper.get_result_callback()
            unreachable = ansible_callback.get_json_result_unreachable()
            if unreachable:
                raise NetHandlerException(f'network service {service_name} was not committed due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.get_logger())
                raise NetHandlerException(f'network service {service_name} was not committed due to config error')
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
                raise NetHandlerException(f"Unit # {unit} has no assigned slivers")

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
        data = {
            "tailf-ncs:services": {
                f'{service_type}:{service_type}': [{
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
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise NetHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
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
            self.get_logger().error(f"Exception occurred in cleanup {e}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e

    def __l2bridge_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
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
            if labs.vlan is None:
                interface['outervlan'] = 0
            else:
                interface['outervlan'] = labs.vlan
            if int(interface['outervlan']) > 0 and labs.inner_vlan is not None:
                interface['innervlan'] = labs.inner_vlan
            if caps is not None and caps.bw is not None and caps.bw != 0:
                interface['bandwidth'] = caps.bw # default unit = gbps
                if caps.burst_size is not None and caps.burst_size != 0:
                    interface['burst-size'] = caps.burst_size # default unit = mbytes
            interfaces.append(interface)
        if not interfaces:
            raise NetHandlerException(f'l2bridge - none valid interface is defined in sliver')
        return data

    def __l2ptp_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        stp_list = []
        if len(sliver.interface_info.interfaces) != 2:
            raise NetHandlerException(
                f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            stp = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise NetHandlerException(f'l2ptp - interface "{interface_name}" has no "device_name" label')
            stp['device'] = labs.device_name
            interface = {}
            if labs.local_name is None:
                raise NetHandlerException(f'l2ptp - interface "{interface_name}" has no "local_name" label')
            interface_type_id = re.findall(r'(\w+)(\d.+)', labs.local_name)
            if not interface_type_id or len(interface_type_id[0]) != 2:
                raise NetHandlerException(f'l2ptp - interface "{interface_name}" has malformed "local_name" label')
            interface['type'] = interface_type_id[0][0]
            interface['id'] = interface_type_id[0][1]
            if labs.vlan is None or labs.vlan == 0:
                raise NetHandlerException(f'l2ptp - interface "{interface_name}" must be tagged (with vlan label in 1..4095)')
            interface['outervlan'] = labs.vlan
            if labs.inner_vlan is not None:
                interface['innervlan'] = labs.inner_vlan

            if caps is not None and caps.bw is not None and caps.bw != 0:
                interface['bandwidth'] = caps.bw
                if caps.burst_size is not None and caps.burst_size != 0:
                    interface['burst-size'] = caps.burst_size
            stp['interface'] = interface
            stp_list.append(stp)

        if stp_list[0]['device'] == stp_list[1]['device']:
            raise NetHandlerException(
                f'l2ptp - has two interfaces on the same device "{stp_list[0]["device"]}" (required to be different)')

        data = {"name": service_name, "stp-a": stp_list[0], "stp-z": stp_list[1]}

        if sliver.ero is not None and len(sliver.ero.get()) == 2:
            ero_a2z = []
            type, path = sliver.ero.get()
            index = 1
            for hop in path.get()[0]:
                # TODO: validate hop is ipv4 using regex
                ero_a2z.append({'index': str(index), 'address': hop})
                index += 1
            hop_a2z = {"hop": ero_a2z}
            data['ero-a2z'] = hop_a2z

            ero_z2a = []
            index = 1
            for hop in path.get()[1]:
                # TODO: validate hop is ipv4 using regex
                ero_z2a.append({'index': str(index), 'address': hop})
                index += 1
            hop_z2a = {"hop": ero_z2a}
            data['ero-z2a'] = hop_z2a
        else:
            self.get_logger().info(f"l2ptp - created without ERO")
        return data

    def __l2sts_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        site_a = {}
        site_z = {}
        if len(sliver.interface_info.interfaces) < 2:
            raise NetHandlerException(
                f'l2sts - sliver requires at least 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            interfaces = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise NetHandlerException(f'l2sts - interface "{interface_name}" has no "device_name" label')
            interface = {}
            if labs.local_name is None:
                raise NetHandlerException(f'l2sts - interface "{interface_name}" has no "local_name" label')
            interface_type_id = re.findall(r'(\w+)(\d.+)', labs.local_name)
            if not interface_type_id or len(interface_type_id[0]) != 2:
                raise NetHandlerException(f'l2sts - interface "{interface_name}" has malformed "local_name" label')
            interface['type'] = interface_type_id[0][0]
            interface['id'] = interface_type_id[0][1]
            if labs.vlan is None:
                interface['outervlan'] = 0
            else:
                interface['outervlan'] = labs.vlan
            if int(interface['outervlan']) > 0 and labs.inner_vlan is not None:
                interface['innervlan'] = labs.inner_vlan

            if caps is not None and caps.bw is not None and caps.bw != 0:
                interface['bandwidth'] = caps.bw
                if caps.burst_size is not None and caps.burst_size != 0:
                    interface['burst-size'] = caps.burst_size

            if not site_a:
                site_a['device'] = labs.device_name
                site_a['interface'] = [interface]
            elif site_a['device'] == labs.device_name:
                site_a['interface'].append(interface)
            elif not site_z:
                site_z['device'] = labs.device_name
                site_z['interface'] = [interface]
            elif site_z['device'] == labs.device_name:
                site_z['interface'].append(interface)
            else:
                raise NetHandlerException(
                    f'l2sts - more than 2 sites are present in the list of interfaces (requires exactly 2)')

        if not site_a or not site_z:
            raise NetHandlerException(
                f'l2sts - fewer than 2 sites are present in the list of interfaces (requires exactly 2)')

        data = {"name": service_name, "site-a": site_a, "site-z": site_z}

        if sliver.ero is not None and len(sliver.ero.get()) == 2:
            ero_a2z = []
            type, path = sliver.ero.get()
            index = 1
            for hop in path.get()[0]:
                # TODO: validate hop is ipv4 using regex
                ero_a2z.append({'index': str(index), 'address': hop})
                index += 1
            hop_a2z = {"hop": ero_a2z}
            data['ero-a2z'] = hop_a2z

            ero_z2a = []
            index = 1
            for hop in path.get()[1]:
                # TODO: validate hop is ipv4 using regex
                ero_z2a.append({'index': str(index), 'address': hop})
                index += 1
            hop_z2a = {"hop": ero_z2a}
            data['ero-z2a'] = hop_z2a
        else:
            self.get_logger().info(f"l2ptp - created without ERO")
        return data
