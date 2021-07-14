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
import threading
import traceback
from typing import Tuple, List

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
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
        Create a VM
        :param unit: unit representing VM
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        unit_id = None

        try:
            self.logger.info(f"Create invoked for unit: {unit}")
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

            if worker_node is None or flavor is None or vmname is None or image is None:
                raise VmHandlerException(f"Missing required parameters workernode: {worker_node} "
                                         f"flavor: {flavor}: vmname: {vmname} image: {image}")

            resource_type = str(sliver.get_type())
            playbook_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # create VM
            playbook_path_full = f"{playbook_path}/{playbook}"
            instance_props = self.__create_vm(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                              vm_name=vmname, image=image, flavor=flavor, worker_node=worker_node,
                                              unit_id=unit_id, ssh_key=ssh_key)

            # Attach FIP
            fip_props = self.__attach_fip(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                          vm_name=vmname, unit_id=unit_id)

            sliver.label_allocations.instance = instance_props.get(AmConstants.SERVER_INSTANCE_NAME, None)

            # Attach any attached PCI Devices
            if sliver.attached_components_info is not None:
                for component in sliver.attached_components_info.devices.values():
                    resource_type = str(component.get_type())
                    playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
                    if playbook is None:
                        raise VmHandlerException(f"Missing parameters playbook: {playbook} "
                                                 f"resource_type: {resource_type} component: {component} "
                                                 f"sliver: {sliver}")

                    playbook_path_full = f"{playbook_path}/{playbook}"
                    self.logger.debug(f"Attaching Devices {playbook_path_full}")
                    self.__attach_detach_pci(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                             host=worker_node, instance_name=sliver.label_allocations.instance,
                                             device_name=str(unit.get_id()),
                                             pci_devices=component.label_allocations.bdf)

            sliver.management_ip = fip_props.get(AmConstants.FLOATING_IP, None)

        except Exception as e:
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver, unit_id=unit_id)
                unit.sliver.label_allocations.instance = None

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:

            self.logger.info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
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
            self.logger.info(f"Delete invoked for unit: {unit}")
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
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Delete completed")
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

        sliver = None
        playbook_path = None
        inventory_path = None
        worker_node = None
        instance_name = None
        try:
            self.logger.info(f"Modify invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            worker_node = sliver.label_allocations.instance_parent
            vmname = sliver.get_name()
            instance_name = sliver.label_allocations.instance

            if worker_node is None or vmname is None or instance_name is None:
                raise VmHandlerException(f"Missing required parameters workernode: {worker_node} "
                                         f"vmname: {vmname} instance_name: {instance_name}")

            playbook_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if sliver.attached_components_info is not None:
                for device in sliver.attached_components_info.devices.values():
                    resource_type = str(device.get_type())
                    playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
                    if playbook is None or inventory_path is None:
                        raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                                 f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
                    full_playbook_path = f"{playbook_path}/{playbook}"
                    self.logger.debug(f"Attaching/Detaching Devices {full_playbook_path}")
                    self.__attach_detach_pci(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                             instance_name=instance_name, host=worker_node,
                                             device_name=str(unit.get_id()),
                                             pci_devices=device.label_allocations.bdf)

        except Exception as e:
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            if sliver is not None and sliver.attached_components_info is not None and inventory_path is not None and \
                    playbook_path is not None:
                for device in sliver.attached_components_info.devices.values():
                    resource_type = str(device.get_type())
                    playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
                    if playbook is None or inventory_path is None:
                        raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                                 f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
                    full_playbook_path = f"{playbook_path}/{playbook}"
                    self.logger.debug(f"Detaching Devices {full_playbook_path}")
                    self.__attach_detach_pci(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                             instance_name=instance_name, host=worker_node,
                                             device_name=str(unit.get_id()),
                                             pci_devices=device.label_allocations.bdf, attach=False)

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:

            self.logger.info(f"Modify completed")
        return result, unit

    def __create_vm(self, *, playbook_path: str, inventory_path: str, vm_name: str,
                    worker_node: str, image: str, flavor: str, unit_id: str, ssh_key: str) -> dict:
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
        :return: dictionary containing created instance details
        """
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        vm_name_combined = f"{unit_id}-{vm_name}"

        avail_zone = f"nova:{worker_node}"

        default_user = self.__get_default_user(image=image)

        extra_vars = {
            AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_CREATE,
            AmConstants.EC2_AVAILABILITY_ZONE: avail_zone,
            AmConstants.VM_NAME: vm_name_combined,
            AmConstants.FLAVOR: flavor,
            AmConstants.IMAGE: image,
            AmConstants.HOSTNAME: vm_name,
            AmConstants.SSH_KEY: ssh_key,
            AmConstants.DEFAULT_USER: default_user
        }
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.logger.debug(f"Executing playbook {playbook_path} to create VM")
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
        self.logger.debug(f"Returning properties {result}")

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
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_DELETE,
                      AmConstants.VM_NAME: vm_name}
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.logger.debug(f"Executing playbook {playbook_path} to delete VM")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        return True

    def __attach_fip(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str) -> dict:
        """
        Invoke ansible playbook to attach a floating IP to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :return: dictionary containing created floating ip details
        """
        vm_name = f"{unit_id}-{vm_name}"
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_ATTACH_FIP,
                      AmConstants.VM_NAME: vm_name}
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.logger.debug(f"Executing playbook {playbook_path} to attach FIP")
        ansible_helper.run_playbook(playbook_path=playbook_path)

        ok = ansible_helper.get_result_callback().get_json_result_ok()

        floating_ip = ok.get(AmConstants.FLOATING_IP, None)
        # Added this code for enabling test suite
        if floating_ip is None:
            floating_ip = ok[AmConstants.ANSIBLE_FACTS][AmConstants.FLOATING_IP]
            floating_ip = json.loads(floating_ip)

        result = {AmConstants.FLOATING_IP: str(floating_ip[AmConstants.FLOATING_IP_ADDRESS]),
                  AmConstants.FLOATING_IP_MAC_ADDRESS: str(floating_ip[AmConstants.FLOATING_IP_PROPERTIES][
                      AmConstants.FLOATING_IP_PORT_DETAILS][AmConstants.FLOATING_IP_MAC_ADDRESS])}

        self.logger.debug(f"Returning properties {result}")
        return result

    def __attach_detach_pci(self, *, playbook_path: str, inventory_path: str, host: str, instance_name: str,
                            device_name:str, pci_devices: str, attach: bool = True):
        """
        Invoke ansible playbook to attach/detach a PCI device to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param instance_name: Instance Name
        :param device_name: Device Name
        :param pci_devices: PCI Device
        :param attach: True for attach and False for detach
        :return:
        """
        self.logger.debug("__attach_detach_pci IN")
        try:
            pci_device_list = None
            if isinstance(pci_devices, str):
                pci_device_list = [pci_devices]
            else:
                pci_device_list = pci_devices

            worker_node = host

            extra_vars = {AmConstants.WORKER_NODE_NAME: worker_node,
                          AmConstants.PCI_PROV_DEVICE: device_name}
            if attach:
                extra_vars[AmConstants.PCI_OPERATION] = AmConstants.PCI_PROV_ATTACH
            else:
                extra_vars[AmConstants.PCI_OPERATION] = AmConstants.PCI_PROV_DETACH

            self.logger.debug(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            for device in pci_device_list:
                ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
                ansible_helper.set_extra_vars(extra_vars=extra_vars)

                device_char_arr = self.__extract_device_addr_octets(device_address=device)
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.KVM_GUEST_NAME, value=instance_name)
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_DOMAIN, value=device_char_arr[0])
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_BUS, value=device_char_arr[1])
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_SLOT, value=device_char_arr[2])
                ansible_helper.add_vars(host=worker_node, var_name=AmConstants.PCI_FUNCTION, value=device_char_arr[3])

                self.logger.debug(f"Executing playbook {playbook_path} to attach({attach})/detach({not attach}) PCI device "
                                  f"({device})")

                ansible_helper.run_playbook(playbook_path=playbook_path)
        finally:
            self.logger.debug("__attach_detach_pci OUT")

    def __cleanup(self, *, sliver: NodeSliver, unit_id: str, raise_exception: bool = False):
        """
        Cleanup VM and detach PCI devices
        :param sliver: Sliver
        :param unit_id: Unit Id
        :param raise_exception: Raise exception if raise_exception flag is True
        :return:
        """
        try:
            playbook_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.config[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            worker_node = sliver.label_allocations.instance_parent
            instance_name = sliver.label_allocations.instance

            if inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook_path: {playbook_path} "
                                         f"inventory_path: {inventory_path}")

            if sliver.attached_components_info is not None and worker_node is not None and instance_name is not None:
                for device in sliver.attached_components_info.devices.values():
                    try:
                        resource_type = str(device.get_type())
                        playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
                        if playbook is None or inventory_path is None:
                            raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                                     f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
                        full_playbook_path = f"{playbook_path}/{playbook}"

                        if device.label_allocations.bdf is None:
                            raise VmHandlerException(f"Missing required parameters bdf: {device.label_allocations.bdf}")
                        self.logger.debug(f"Attaching/Detaching Devices {full_playbook_path}")
                        self.__attach_detach_pci(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                                 instance_name=instance_name, host=worker_node,
                                                 device_name=unit_id,
                                                 pci_devices=device.label_allocations.bdf,
                                                 attach=False)
                    except Exception as e:
                        self.logger.error(f"Error occurred detaching device: {device}")
                        if raise_exception:
                            raise e

            resource_type = str(sliver.get_type())
            playbook = self.config[AmConstants.PLAYBOOK_SECTION][resource_type]
            full_playbook_path = f"{playbook_path}/{playbook}"

            if sliver.get_name() is None:
                raise VmHandlerException(f"Missing required parameters vm_name: {sliver.get_name()}")

            # Delete VM
            self.__delete_vm(playbook_path=full_playbook_path, inventory_path=inventory_path,
                             vm_name=sliver.get_name(), unit_id=unit_id)
        except Exception as e:
            self.logger.error(f"Exception occurred in cleanup {e}")
            self.logger.error(traceback.format_exc())
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

    def __get_vm(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str) -> bool:
        """
        Invoke ansible playbook to get a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :return: True or False representing success/failure
        """
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.logger)
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_GET,
                      AmConstants.VM_NAME: vm_name}
        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        self.logger.debug(f"Executing playbook {playbook_path} to get VM")
        ansible_helper.run_playbook(playbook_path=playbook_path)
        return True

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
