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
import json
import re
import traceback
from typing import Tuple, List

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.attached_components import ComponentSliver, ComponentType
from fim.slivers.network_node import NodeSliver

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class VmHandlerException(Exception):
    """
    VM Handler Exception
    """
    pass


class VMHandler(HandlerBase):
    """
    VM Handler
    """
    DEFAULT_USERS = [AmConstants.FEDORA_DEFAULT_USER, AmConstants.CENTOS_DEFAULT_USER, AmConstants.UBUNTU_DEFAULT_USER,
                     AmConstants.DEBIAN_DEFAULT_USER]
    test_mode = False

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
            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()
            ssh_key = unit_properties.get(Constants.USER_SSH_KEY, None)

            worker_node = sliver.label_allocations.instance_parent
            flavor = sliver.get_capacity_hints().instance_type
            vmname = sliver.get_name()
            image = sliver.get_image_ref()
            init_script = None
            # TODO uncomment when FIM change is available
            #init_script = sliver.label_allocations.init_script

            if worker_node is None or flavor is None or vmname is None or image is None:
                raise VmHandlerException(f"Missing required parameters workernode: {worker_node} "
                                         f"flavor: {flavor}: vmname: {vmname} image: {image}")

            resource_type = str(sliver.get_type())
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # create VM
            playbook_path_full = f"{playbook_path}/{playbook}"
            instance_props = self.__create_vm(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                              vm_name=vmname, image=image, flavor=flavor, worker_node=worker_node,
                                              unit_id=unit_id, ssh_key=ssh_key, init_script=init_script)

            disable_fip = self.get_config()[AmConstants.RUNTIME_SECTION][AmConstants.RT_DISABLE_FIP]

            if disable_fip:
                self.get_logger().info("Floating IP is disabled, using IPV6 Global Unicast Address")
                fip = instance_props.get(AmConstants.SERVER_ACCESS_IPV6, None)
            else:
                # Attach FIP
                fip = self.__attach_fip(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                        vm_name=vmname, unit_id=unit_id)

            sliver.label_allocations.instance = instance_props.get(AmConstants.SERVER_INSTANCE_NAME, None)

            user = self.__get_default_user(image=image)

            # Attach any attached PCI Devices
            if sliver.attached_components_info is not None:
                for component in sliver.attached_components_info.devices.values():
                    resource_type = str(component.get_type())
                    playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
                    if playbook is None:
                        raise VmHandlerException(f"Missing parameters playbook: {playbook} "
                                                 f"resource_type: {resource_type} component: {component} "
                                                 f"sliver: {sliver}")

                    playbook_path_full = f"{playbook_path}/{playbook}"
                    self.get_logger().debug(f"Attaching Devices {playbook_path_full}")
                    self.__attach_detach_pci(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                             host=worker_node, instance_name=sliver.label_allocations.instance,
                                             device_name=str(unit.get_id()), component=component, user=user,
                                             mgmt_ip=fip)

            sliver.management_ip = fip

        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver, unit_id=unit_id)
                unit.sliver.label_allocations.instance = None

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
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

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
        """
        Modify a provisioned Unit
        :param unit: unit representing VM
        :param properties: properties
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Modify invoked for unit: {unit}")
            # TODO
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            # TODO
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:

            self.get_logger().info(f"Modify completed")
        return result, unit

    def __create_vm(self, *, playbook_path: str, inventory_path: str, vm_name: str, worker_node: str, image: str,
                    flavor: str, unit_id: str, ssh_key: str, init_script: str = None) -> dict:
        """
        Invoke ansible playbook to provision a VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param vm_name: VM Name
        :param worker_node: worker_node
        :param image: Image
        :param flavor: Flavor
        :param unit_id: Unit Id
        :param ssh_key: ssh_key
        :param init_script: Init Script
        :return: dictionary containing created instance details
        """
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
        vm_name_combined = f"{unit_id}-{vm_name}"

        avail_zone = f"nova:{worker_node}"

        default_user = self.__get_default_user(image=image)

        if init_script is None:
            init_script = ""

        extra_vars = {
            AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_CREATE,
            AmConstants.EC2_AVAILABILITY_ZONE: avail_zone,
            AmConstants.VM_NAME: vm_name_combined,
            AmConstants.FLAVOR: flavor,
            AmConstants.IMAGE: image,
            AmConstants.HOSTNAME: vm_name,
            AmConstants.SSH_KEY: ssh_key,
            AmConstants.DEFAULT_USER: default_user,
            AmConstants.INIT_SCRIPT: init_script
        }
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.get_logger().debug(f"Executing playbook {playbook_path} to create VM extra_vars: {extra_vars}")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        ok = ansible_helper.get_result_callback().get_json_result_ok()

        server = ok.get(AmConstants.SERVER, None)
        # Added this code for enabling test suite
        if server is None:
            server = ok[AmConstants.ANSIBLE_FACTS][AmConstants.SERVER]
            server = json.loads(server)

        result = {
            AmConstants.SERVER_VM_STATE: str(server[AmConstants.SERVER_VM_STATE]),
            AmConstants.SERVER_INSTANCE_NAME: str(server[AmConstants.SERVER_INSTANCE_NAME]),
            AmConstants.SERVER_ACCESS_IPV4: str(server[AmConstants.SERVER_ACCESS_IPV4])
        }
        if server[AmConstants.SERVER_ACCESS_IPV6] is not None:
            result[AmConstants.SERVER_ACCESS_IPV6] = str(server[AmConstants.SERVER_ACCESS_IPV6])
        self.get_logger().debug(f"Returning properties {result}")

        return result

    def __delete_vm(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str) -> bool:
        """
        Invoke ansible playbook to remove a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :return: True or False representing success/failure
        """
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_DELETE,
                      AmConstants.VM_NAME: vm_name}
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.get_logger().debug(f"Executing playbook {playbook_path} to delete VM extra_vars: {extra_vars}")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        return True

    def __attach_fip(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str) -> str:
        """
        Invoke ansible playbook to attach a floating IP to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :return: floating ip assigned to the VM
        """
        try:
            self.process_lock.acquire()
            vmname = f"{unit_id}-{vm_name}"
            ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
            extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_ATTACH_FIP,
                          AmConstants.VM_NAME: vmname}
            ansible_helper.set_extra_vars(extra_vars=extra_vars)

            self.get_logger().debug(f"Executing playbook {playbook_path} to attach FIP extra_vars: {extra_vars}")
            ansible_helper.run_playbook(playbook_path=playbook_path)

            ok = ansible_helper.get_result_callback().get_json_result_ok()

            if self.test_mode:
                floating_ip = ok[AmConstants.ANSIBLE_FACTS][AmConstants.FLOATING_IP]
                floating_ip = json.loads(floating_ip)
                return floating_ip[AmConstants.FLOATING_IP_ADDRESS]

            floating_ip = ok[AmConstants.FLOATING_IP]
            result = None
            if floating_ip is None:
                self.get_logger().info("Floating IP returned by attach was null, trying to get via get_vm")
                ok = self.__get_vm(playbook_path=playbook_path, inventory_path=inventory_path, vm_name=vm_name,
                                   unit_id=unit_id)
                self.get_logger().info(f"Info returned by __get_vm: {ok}")
                servers = ok[AmConstants.OS_SERVERS]
                self.get_logger().debug(f"Servers: {servers}")
                if servers is not None and len(servers) == 1:
                    result = servers[0][AmConstants.SERVER_ACCESS_IPV4]
                    if result is None:
                        result = servers[0][AmConstants.SERVER_ACCESS_IPV6]
                else:
                    self.get_logger().error(f"No server found for {unit_id}-{vm_name}")
            else:
                result = floating_ip[AmConstants.FLOATING_IP_ADDRESS]

            if result is None:
                raise VmHandlerException(f"Unable to get the Floating IP for {unit_id}-{vm_name}")

            self.get_logger().debug(f"Returning FIP {result} for {unit_id}-{vm_name}")
            return str(result)
        finally:
            self.process_lock.release()

    def __attach_detach_pci(self, *, playbook_path: str, inventory_path: str, host: str, instance_name: str,
                            device_name: str, component: ComponentSliver, user: str = None, mgmt_ip: str = None,
                            attach: bool = True):
        """
        Invoke ansible playbook to attach/detach a PCI device to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param instance_name: Instance Name
        :param device_name: Device Name
        :param component: Component Sliver
        :param mgmt_ip: Management IP of the VM to which the component is attached
        :param attach: True for attach and False for detach
        :return:
        """
        self.get_logger().debug("__attach_detach_pci IN")
        try:
            pci_device_list = None
            if isinstance(component.label_allocations.bdf, str):
                pci_device_list = [component.label_allocations.bdf]
            else:
                pci_device_list = component.label_allocations.bdf

            worker_node = host

            extra_vars = {AmConstants.WORKER_NODE_NAME: worker_node,
                          AmConstants.PCI_PROV_DEVICE: device_name}
            if attach:
                extra_vars[AmConstants.PCI_OPERATION] = AmConstants.PCI_PROV_ATTACH
            else:
                extra_vars[AmConstants.PCI_OPERATION] = AmConstants.PCI_PROV_DETACH

            self.get_logger().debug(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            index = 0
            for device in pci_device_list:
                ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
                ansible_helper.set_extra_vars(extra_vars=extra_vars)

                device_char_arr = self.__extract_device_addr_octets(device_address=device)
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.KVM_GUEST_NAME, value=instance_name)
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_DOMAIN, value=device_char_arr[0])
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_BUS, value=device_char_arr[1])
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_SLOT, value=device_char_arr[2])
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_FUNCTION, value=device_char_arr[3])

                self.get_logger().debug(f"Executing playbook {playbook_path} to attach({attach})/detach({not attach}) "
                                        f"PCI device ({device}) extra_vars: {extra_vars}")

                ansible_helper.run_playbook(playbook_path=playbook_path)

            # Configure the Network Interface card
            if attach:
                self.configure_nic(component=component, mgmt_ip=mgmt_ip, user=user)
        finally:
            self.get_logger().debug("__attach_detach_pci OUT")

    def __cleanup(self, *, sliver: NodeSliver, unit_id: str, raise_exception: bool = False):
        """
        Cleanup VM and detach PCI devices
        :param sliver: Sliver
        :param unit_id: Unit Id
        :param raise_exception: Raise exception if raise_exception flag is True
        :return:
        """
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            worker_node = sliver.label_allocations.instance_parent
            instance_name = sliver.label_allocations.instance

            if inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook_path: {playbook_path} "
                                         f"inventory_path: {inventory_path}")

            if sliver.attached_components_info is not None and worker_node is not None and instance_name is not None:
                for device in sliver.attached_components_info.devices.values():
                    try:
                        resource_type = str(device.get_type())
                        playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
                        if playbook is None or inventory_path is None:
                            raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                                     f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
                        full_playbook_path = f"{playbook_path}/{playbook}"

                        if device.label_allocations.bdf is None:
                            raise VmHandlerException(f"Missing required parameters bdf: {device.label_allocations.bdf}")
                        self.get_logger().debug(f"Attaching/Detaching Devices {full_playbook_path}")
                        self.__attach_detach_pci(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                                 instance_name=instance_name, host=worker_node,
                                                 device_name=unit_id,
                                                 component=device,
                                                 attach=False)
                    except Exception as e:
                        self.get_logger().error(f"Error occurred detaching device: {device}")
                        if raise_exception:
                            raise e

            resource_type = str(sliver.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            full_playbook_path = f"{playbook_path}/{playbook}"

            if sliver.get_name() is None:
                raise VmHandlerException(f"Missing required parameters vm_name: {sliver.get_name()}")

            # Delete VM
            self.__delete_vm(playbook_path=full_playbook_path, inventory_path=inventory_path,
                             vm_name=sliver.get_name(), unit_id=unit_id)
        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup {e}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e

    @staticmethod
    def __extract_device_addr_octets(*, device_address: str) -> List[str]:
        """
        Function to extract PCI domain, bus, slot and function from BDF
        :param device_address BDF
        :return list containing PCI domain, bus, slot and function from BDF
        """
        match = re.split("(.*):(.*):(.*)\\.(.*)", device_address)
        result = []
        match = match[1:-1]
        for octet in match:
            octet = octet.lstrip("0")
            if octet == "":
                octet = '0x0'
            else:
                octet = f"0x{octet}"

            result.append(octet)
        return result

    def __get_vm(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str):
        """
        Invoke ansible playbook to get a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :return: OK result
        """
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger())
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_GET,
                      AmConstants.VM_NAME: vm_name}
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.get_logger().debug(f"Executing playbook {playbook_path} to get VM extra_vars: {extra_vars}")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        return ansible_helper.get_result_callback().get_json_result_ok()

    @staticmethod
    def __get_default_user(image: str) -> str:
        """
        Return default SSH user name
        :return default ssh user name
        """
        for user in VMHandler.DEFAULT_USERS:
            if user in image:
                return user
        return AmConstants.ROOT_USER

    def configure_nic(self, *, component: ComponentSliver, mgmt_ip: str, user: str):
        """
        Configure Interfaces associated with SharedNIC or SmartNIC
        :param component Component Sliver
        :param mgmt_ip Management IP
        :param user Default Linux user for the VM
        """
        # Only do this for SharedNIC and SmartNIC
        if component.get_type() == ComponentType.SharedNIC and component.get_type() != ComponentType.SmartNIC:
            return

        if component.network_service_info is None or component.network_service_info.network_services is None:
            return

        for ns in component.network_service_info.network_services.values():
            if ns.interface_info is None or ns.interface_info.interfaces is None:
                continue

            for ifs in ns.interface_info.interfaces.values():
                self.get_logger().info(f"Configuring Interface  {ifs}")
                self.configure_network_interface(mgmt_ip=mgmt_ip, user=user, resource_type=component.get_type().name,
                                                 ipv4_address=ifs.label_allocations.ipv4,
                                                 ipv6_address=ifs.label_allocations.ipv6,
                                                 mac_address=ifs.label_allocations.mac,
                                                 vlan=ifs.label_allocations.vlan)

    def configure_network_interface(self, *, mgmt_ip: str, user: str, resource_type: str, mac_address: str,
                                    ipv4_address: str = None, ipv6_address: str = None, vlan: str = None):
        """
        Configure Network Interface inside the VM
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param resource_type Type of NIC card (SharedNIC or SmartNIC)
        :param mac_address Mac address used to identify the interface
        :param ipv4_address IPV4 address to assign
        :param ipv6_address IPV6 address to assign
        :param vlan Vlan tag in case of tagged interface
        """
        try:
            if ipv6_address is None and ipv4_address is None:
                return False

            # Grab the playbook location
            playbook_location = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]

            # Grab the playbook name
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CONFIG][resource_type]

            # Construct the playbook path
            playbook_path = f"{playbook_location}/{playbook}"

            # Construct ansible helper
            ansible_helper = AnsibleHelper(inventory_path=None, logger=self.get_logger(), sources=f"{mgmt_ip},")

            # Set the variables
            extra_vars = {AmConstants.VM_NAME: mgmt_ip,
                          AmConstants.MAC: mac_address.lower(),
                          AmConstants.IMAGE: user}

            if ipv4_address is not None:
                extra_vars[AmConstants.IPV4_ADDRESS] = ipv4_address
            if ipv6_address is not None:
                extra_vars[AmConstants.IPV6_ADDRESS] = ipv6_address
            if vlan is not None and resource_type != ComponentType.SharedNIC.name:
                extra_vars[AmConstants.VLAN] = vlan

            ansible_helper.set_extra_vars(extra_vars=extra_vars)

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]

            # Invoke the playbook
            self.get_logger().debug(f"Executing playbook {playbook_path} to configure interface extra_vars: "
                                    f"{extra_vars}")
            ansible_helper.run_playbook(playbook_path=playbook_path, user=user, private_key_file=admin_ssh_key)
        except Exception as e:
            self.get_logger().error(f"Exception : {e}")
            self.get_logger().error(traceback.format_exc())
