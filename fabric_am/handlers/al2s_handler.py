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
from fabric_cf.actor.core.util.utils import ns_sliver_to_str
from fabric_cf.actor.core.util.utils import ns_sliver_to_str

from fim.slivers.capacities_labels import Labels, Capacities
from fim.slivers.network_service import NetworkServiceSliver, MirrorDirection, NSLayer

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class Entity():
    """This is the base class for the virutal network classes. It provides json serialization methods."""

    @classmethod
    def from_json(cls, data):
        jsonObj = json.loads(data)
        return cls(**jsonObj)

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=2)

class L3Connection(Entity):
    def __init__(
            self,
            virtualRouterId: str = None,
            interfaceId: str = None,
            remoteName: str = None,
            notes: str = None,
            displayPosition: int = 1,
            encapsulationType: str = None,
            vlanOuterId: int = 0,
            vlanInnerId: int = 0,
            ipv6PrefixLength: int = 0,
            localIPv6: str = None,
            remoteIPv6: str = None,
            ipv4PrefixLength: int = 0,
            localIPv4: str = None,
            remoteIPv4: str = None,
            remoteASN: int = 0,
            authnType: str = None,
            authnConfig: dict = None,
            mtu: int = 9001,
            bfdEnable: bool = True,
            gtsmEnable: bool = True,
            cloudConnectionType: str = None,
            cloudConnectionConfig: dict = None,
            maxBandwidth: int = 0,
            authoringState: str = None
            ):
        self.virtualRouterId = virtualRouterId
        self.interfaceId = interfaceId
        self.remoteName = remoteName
        self.notes = notes
        self.displayPosition = displayPosition
        self.encapsulationType = encapsulationType
        self.vlanOuterId = vlanOuterId
        self.vlanInnerId = vlanInnerId
        self.ipv6PrefixLength = ipv6PrefixLength
        self.localIPv6 = localIPv6
        self.remoteIPv6 = remoteIPv6
        self.ipv4PrefixLength = ipv4PrefixLength
        self.localIPv4 = localIPv4
        self.remoteIPv4 = remoteIPv4
        self.remoteASN = remoteASN
        self.authnType = authnType
        self.authnConfig = authnConfig
        self.mtu = mtu
        self.bfdEnable = bfdEnable
        self.gtsmEnable = gtsmEnable
        self.cloudConnectionType = cloudConnectionType
        self.cloudConnectionConfig = cloudConnectionConfig
        self.maxBandwidth = maxBandwidth
        self.authoringState = authoringState
    
class Al2sHandlerException(Exception):
    """
    AL2S Handler Exception
    """
    pass


class Al2sHandler(HandlerBase):
    """
    AL2S Handler
    """
    def __has_sliver_changed(self, current: NetworkServiceSliver, requested: NetworkServiceSliver):
        diff = current.diff(other_sliver=requested)
        if diff is None:
            return False

        if (diff.added.interfaces is not None and len(diff.added.interfaces)) or \
                (diff.removed.interfaces is not None and len(diff.removed.interfaces)) or \
                (diff.modified.interfaces is not None and len(diff.modified.interfaces)):
            return True

        if diff.modified is not None and diff.modified.services is not None:
            for new_ns, flag in diff.modified.services:
                if flag & WhatsModifiedFlag.LABELS or flag & WhatsModifiedFlag.CAPACITIES:
                    return True
        return False
    
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
        service_type = None
        service_data = None
        try:
            # self.get_logger().info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise Al2sHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()  # use: TBD

            resource_type = str(sliver.get_type())

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise Al2sHandlerException(f"Missing config parameters playbook: {playbook} "
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
                raise Al2sHandlerException(f'unrecognized network service type "{service_type}"')

            extra_vars = service_data
            
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger(),
                                           ansible_python_interpreter=self.get_ansible_python_interpreter())
            ansible_helper.set_extra_vars(extra_vars=extra_vars)
            self.get_logger().debug(f"Executing playbook {playbook_path_full} to create Network Service")
            ansible_helper.run_playbook(playbook_path=playbook_path_full)
            ansible_callback = ansible_helper.get_result_callback()
            unreachable = ansible_callback.get_json_result_unreachable()
            if unreachable:
                raise Al2sHandlerException(f'network service {service_name} was not committed due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.get_logger())
                raise Al2sHandlerException(f'network service {service_name} was not committed due to config error')
            ok = ansible_callback.get_json_result_ok()
            if ok:
                if not ok['changed']:
                    self.get_logger().info(f'network service {service_name} was committed ok but without change')

        except Exception as e:
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                try:
                    self.__cleanup(sliver=sliver, raise_exception=True, unit_id=unit_id)
                except Exception as ex:
                    self.get_logger().error(ex)
                finally:
                    self.get_logger().info(f"Tried to delete VM duo to failure")

            ifs = []
            if service_type is not None and service_data is not None:
                if service_type == 'l2ptp':
                    ifs = [arg['interfaceId'] for arg in service_data['opargs'] if 'interfaceId' in arg]
                elif service_type == 'l3vpn':
                    ifs = [arg['interfaceId'] for arg in service_data['opargs'] if 'interfaceId' in arg]
                else:
                    ifs = []
            ext_e = Exception(e, ifs)

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: ext_e}
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
                raise Al2sHandlerException(f"Unit # {unit} has no assigned slivers")

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
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        
        try:
            self.get_logger().info(f"Modify invoked for unit: {unit}")
            sliver = unit.get_sliver()
            modified_sliver = unit.get_modified()
            self.get_logger().info(f"Modified sliver: {modified_sliver}")

            if not self.__has_sliver_changed(current=sliver, requested=modified_sliver):
                self.get_logger().info(f"Modify - NO OP")
                return result, unit

            if sliver is None or modified_sliver is None:
                raise NetHandlerException(f"Unit # {unit} has no assigned slivers for modify")

            if not isinstance(sliver, NetworkServiceSliver) or not isinstance(modified_sliver, NetworkServiceSliver):
                raise NetHandlerException(f"Invalid Sliver type {type(sliver)}  {type(modified_sliver)}")

            if sliver.get_type() != modified_sliver.get_type():
                raise NetHandlerException(f"Modify cannot change Sliver type {sliver.get_type()}  into {modified_sliver.get_type()}")

            if sliver.get_name() != modified_sliver.get_name():
                raise NetHandlerException(f"Modify cannot change Sliver name {sliver.get_name()}  into {modified_sliver.get_name()}")

            self.logger.info("Current Sliver:")
            self.logger.info(ns_sliver_to_str(sliver=sliver))

            self.logger.info("Modified Sliver:")
            self.logger.info(ns_sliver_to_str(sliver=modified_sliver))

            unit_id = str(unit.get_reservation_id())

            resource_type = str(modified_sliver.get_type())

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise NetHandlerException(f"Missing config parameters playbook: {playbook} "
                                          f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"

            # same name for sliver and modified_sliver
            if sliver.get_labels() is None or sliver.get_labels().local_name is None:
                # truncate service_name length to no greater than 53 (36+1+16)
                sliver_name = sliver.get_name()[:16] if len(sliver.get_name()) > 16 else sliver.get_name()
                service_name = f'{sliver_name}-{unit_id}'
            else:
                service_name = sliver.get_labels().local_name
            service_type = resource_type.lower()
            if service_type == 'l2bridge':
                service_data = self.__l2bridge_create_data(modified_sliver, service_name)
            elif service_type == 'l2ptp':
                service_data = self.__l2ptp_create_data(modified_sliver, service_name)
            elif service_type == 'l2sts':
                service_data = self.__l2sts_create_data(modified_sliver, service_name)
            elif service_type == 'fabnetv4' or service_type == 'fabnetv4ext':
                service_data = self.__fabnetv4_create_data(modified_sliver, service_name)
                service_type = 'l3rt'
            elif service_type == 'fabnetv6' or service_type == 'fabnetv6ext':
                service_data = self.__fabnetv6_create_data(modified_sliver, service_name)
                service_type = 'l3rt'
            elif service_type == 'l3vpn':
                service_data = self.__l3vpn_modify_data(sliver=sliver, modified_sliver=modified_sliver, service_name=service_name)
                service_type = 'l3vpn'
            elif service_type == 'portmirror':
                service_data = self.__portmirror_create_data(modified_sliver, service_name)
                service_type = 'port-mirror'
            else:
                raise NetHandlerException(f'unrecognized network service type "{service_type}"')

            extra_vars = service_data
            
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
            self.get_logger().error(e)
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info(f"Modify completed")
        
        # Nothing to do at this time
        return result, unit

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
                raise Al2sHandlerException(f"Missing config parameters playbook: {playbook} "
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
                raise Al2sHandlerException(f'network service {service_name} was not cleaned up due to connection error')
            failed = ansible_callback.get_json_result_failed()
            if failed:
                ansible_callback.dump_all_failed(logger=self.get_logger())
                raise Al2sHandlerException(f'network service {service_name} was not cleaned up due to config error')
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
            raise Al2sHandlerException(
                f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise Al2sHandlerException(f'l2ptp - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise Al2sHandlerException(f'l2ptp - interface "{interface_name}" has no "local_name" label')
            if caps is not None and caps.bw is not None:
                endpoint['bandwidth'] = caps.bw * 1000
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
            raise Al2sHandlerException(
                f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L2",
                "description": service_name}
        
        return data

    def __l3vpn_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        
        l3Connections = []
        
        for i, interface_name in enumerate(sliver.interface_info.interfaces, start = 1):
            connection = L3Connection(
                displayPosition = i,
                encapsulationType = "DOT1Q",
                # authnType = "MD5",
                authoringState = "LIVE")
            
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            peerlabs: Labels = interface_sliver.get_peer_labels()
            caps: Capacities = interface_sliver.get_capacities()
            
            if labs.device_name is None:
                raise Al2sHandlerException(f'l3vpn - interface "{interface_name}" has no "device_name" label')
            if labs.local_name is None:
                raise Al2sHandlerException(f'l3vpn - interface "{interface_name}" has no "local_name" label')
            
            if caps is not None and caps.bw is not None:
                connection.maxBandwidth = caps.bw      # specified in Mbps
                
            # connection.device = labs.device_name
            # connection.interface = labs.local_name
            connection.interfaceId = labs.device_name + ":" + labs.local_name
            # connection.interfaceId = "5079eacf-3de6-42ba-a342-832ad8117e6f"
            
            connection.remoteName = peerlabs.local_name
            connection.vlanOuterId = str(labs.vlan)
            connection.mtu = caps.mtu
            
            connection.ipv4PrefixLength = int(labs.ipv4_subnet.split('/')[-1])
            connection.localIPv4 = labs.ipv4_subnet.split('/')[0]
            connection.remoteIPv4 = peerlabs.ipv4_subnet.split('/')[0]
            
            connection.remoteASN = peerlabs.asn
            
            if peerlabs.bgp_key:
                connection.authnType = "MD5"
                connection.authnConfig = {"md5": peerlabs.bgp_key}
            
            if peerlabs.account_id:
                if peerlabs.local_name == 'AWS':
                    connection.cloudConnectionType = 'AWS'
                    connection.cloudConnectionConfig = {"ownerAccountId":peerlabs.account_id}
                elif peerlabs.local_name == 'Google Cloud Platform':
                    connection.cloudConnectionType = 'GCP'
                    connection.cloudConnectionConfig = {"pairingKey":peerlabs.account_id}
                else:
                    raise Exception(f"Unimplemented cloud connect: {peerlabs.local_name}")
            else:
                connection.cloudConnectionType = 'NONCLOUD'
                connection.cloudConnectionConfig = {}
                
            
            # connection.entity = str(sliver.get_name())
            
            l3Connections.append(vars(connection))
        
        data = {"name": service_name,
                "description": service_name,
                "op": "create",
                "level": "L3",
                "opargs": l3Connections}
        
        return data
    
    def _create_connections(self, interfaces:dict):
        """ create a list of connections from the interfaces
        """
        l3Connections = []
        for i, interface_name in enumerate(interfaces, start = 1):
            connection = L3Connection(
                displayPosition = i,
                encapsulationType = "DOT1Q",
                # authnType = "MD5",
                authoringState = "LIVE")
            
            interface_sliver = interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            peerlabs: Labels = interface_sliver.get_peer_labels()
            caps: Capacities = interface_sliver.get_capacities()
            
            if labs.device_name is None:
                raise Al2sHandlerException(f'l3vpn - interface "{interface_name}" has no "device_name" label')
            if labs.local_name is None:
                raise Al2sHandlerException(f'l3vpn - interface "{interface_name}" has no "local_name" label')
            
            if caps is not None and caps.bw is not None:
                connection.maxBandwidth = caps.bw      # specified in Mbps
                
            # connection.device = labs.device_name
            # connection.interface = labs.local_name
            connection.interfaceId = labs.device_name + ":" + labs.local_name
            # connection.interfaceId = "5079eacf-3de6-42ba-a342-832ad8117e6f"
            
            connection.remoteName = peerlabs.local_name
            connection.vlanOuterId = str(labs.vlan)
            connection.mtu = caps.mtu
            
            connection.ipv4PrefixLength = int(labs.ipv4_subnet.split('/')[-1])
            connection.localIPv4 = labs.ipv4_subnet.split('/')[0]
            connection.remoteIPv4 = peerlabs.ipv4_subnet.split('/')[0]
            
            connection.remoteASN = peerlabs.asn
            
            if peerlabs.bgp_key:
                connection.authnType = "MD5"
                connection.authnConfig = {"md5": peerlabs.bgp_key}
            
            if peerlabs.account_id:
                if peerlabs.local_name == 'AWS':
                    connection.cloudConnectionType = 'AWS'
                    connection.cloudConnectionConfig = {"ownerAccountId":peerlabs.account_id}
                elif peerlabs.local_name == 'Google Cloud Platform':
                    connection.cloudConnectionType = 'GCP'
                    connection.cloudConnectionConfig = {"pairingKey":peerlabs.account_id}
                else:
                    raise Exception(f"Unimplemented cloud connect: {peerlabs.local_name}")
            else:
                connection.cloudConnectionType = 'NONCLOUD'
                connection.cloudConnectionConfig = {}
                
            
            # connection.entity = str(sliver.get_name())
            
            l3Connections.append(vars(connection))
        
        return l3Connections
        

    def __l3vpn_modify_data(self, sliver: NetworkServiceSliver,  modified_sliver: NetworkServiceSliver, service_name: str) -> dict:
        
        diff = sliver.diff(modified_sliver)
        
        added_interfaces = {}
        for inf in diff.added.interfaces:
            key = inf.get_name()    # inf -> interfacesliver
            if key in modified_sliver.interface_info.interfaces:
                added_interfaces.update({key:modified_sliver.interface_info.interfaces[key]})
        added_connections = self._create_connections(added_interfaces)
        
        removed_interfaces = {}
        for inf in diff.removed.interfaces:
            key = inf.get_name()    # inf -> interfacesliver
            if key in sliver.interface_info.interfaces:
                removed_interfaces.update({key:sliver.interface_info.interfaces[key]})
        removed_connections = self._create_connections(removed_interfaces)
        
        modified_interfaces = {}
        for inf in diff.modified.interfaces:
            key = inf[0].get_name()     # inf -> tuple
            if key in modified_sliver.interface_info.interfaces:
                modified_interfaces.update({key:modified_sliver.interface_info.interfaces[key]})
        modified_connections = self._create_connections(modified_interfaces)
        
        data = {"name": service_name,
                "description": service_name,
                "op": "modify",
                "level": "L3",
                "opargs_a": added_connections,
                "opargs_m": modified_connections,
                "opargs_r": removed_connections
                }
        
        return data

    def __l3vpn_delete_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise Al2sHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L3",
                "name": service_name}
        
        return data

    def __l2cloud_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise Al2sHandlerException(
        #         f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise Al2sHandlerException(f'l2ptp - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise Al2sHandlerException(f'l2ptp - interface "{interface_name}" has no "local_name" label')
            if caps is not None and caps.bw is not None:
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
        #     raise Al2sHandlerException(
        #         f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L2",
                "description": service_name}
        
        return data

    def __l3cloud_create_data(self, sliver: NetworkServiceSliver, service_name: str) -> dict:
        endpoint_list = []
        local_asn = ''
        # if len(sliver.interface_info.interfaces) != 2:
        #     raise Al2sHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        for interface_name in sliver.interface_info.interfaces:
            endpoint = {}
            interface_sliver = sliver.interface_info.interfaces[interface_name]
            labs: Labels = interface_sliver.get_labels()
            caps: Capacities = interface_sliver.get_capacities()
            if labs.device_name is None:
                raise Al2sHandlerException(f'l3cloud - interface "{interface_name}" has no "device_name" label')
            endpoint['node'] = labs.device_name
            if labs.local_name is None:
                raise Al2sHandlerException(f'l3cloud - interface "{interface_name}" has no "local_name" label')
            if local_asn and  local_asn != labs.asn:
                self.get_logger().error(f"local asn is inconsistant in __l3cloud_create_data")
                raise Al2sHandlerException(f'l3cloud - interface "{interface_name}" has inconsistant local_asn')
            elif not local_asn:
                local_asn = labs.asn

            if caps is not None and caps.bw is not None:
                endpoint['bandwidth'] = caps.bw
            endpoint['interface'] = labs.local_name
            endpoint['tag'] = labs.vlan
            if caps is not None and caps.jumbo is not None:
                endpoint['jumbo'] = caps.jumbo
            endpoint['cloud_account_id'] = labs.account_id
            endpoint['peers'] = {}
            if interface_name in sliver.get_peer_labels():
                endpoint['peers']  =  [sliver.get_peer_labels()[interface_name]]
            else:
                self.get_logger().error(f"Peers not found in __l3cloud_create_data")
                raise Al2sHandlerException(f'l3cloud - interface "{interface_name}" has no peers')
                    
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
        #     raise Al2sHandlerException(
                # f'l2ptp - sliver requires 2 interfaces but was given {len(sliver.interface_info.interfaces)}')

        data = {"op": "delete",
                "level": "L3",
                "name": service_name}
        
        return data
    
    def clean_restart(self):
        pass

    def poa(self, unit: ConfigToken, data: dict) -> Tuple[dict, ConfigToken]:
        """
        Not implemented
        """
        pass
