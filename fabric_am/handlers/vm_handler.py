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
import time
import traceback
from typing import Tuple, List

import paramiko
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
    test_mode = False

    def get_ansible_python_interpreter(self) -> str:
        return self.get_config()[AmConstants.ANSIBLE_SECTION][
                AmConstants.ANSIBLE_PYTHON_INTERPRETER]

    def clean_restart(self):
        self.get_logger().debug("Clean restart - begin")
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            cleanup_section = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CLEANUP]
            cleanup_playbook = f"{playbook_path}/{cleanup_section[AmConstants.CLEAN_ALL]}"
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            extra_vars = {AmConstants.VM_PROV_OP: AmConstants.PROV_OP_DELETE_ALL}
            self.__execute_ansible(inventory_path=inventory_path, playbook_path=cleanup_playbook,
                                   extra_vars=extra_vars)
        except Exception as e:
            self.get_logger().error(f"Failure to clean up existing VMs: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().debug("Clean restart - end")

        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CLEAN_RESTART,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        return result

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
        project_id = None
        try:
            self.get_logger().info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            unit_id = str(unit.get_reservation_id())
            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            unit_properties = unit.get_properties()
            ssh_key = unit_properties.get(Constants.USER_SSH_KEY, None)
            project_id = unit_properties.get(Constants.PROJECT_ID, None)

            worker_node = sliver.label_allocations.instance_parent
            flavor = sliver.get_capacity_hints().instance_type
            vmname = sliver.get_name()
            image = sliver.get_image_ref()
            init_script = sliver.get_boot_script()

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

            user = self.__get_default_user(image=image)

            if disable_fip:
                self.get_logger().info("Floating IP is disabled, using IPV6 Global Unicast Address")
                fip = instance_props.get(AmConstants.SERVER_ACCESS_IPV6, None)
            else:
                # Attach FIP
                fip = self.__attach_fip(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                        vm_name=vmname, unit_id=unit_id)

            ssh_retries = self.get_config()[AmConstants.RUNTIME_SECTION][AmConstants.RT_SSH_RETRIES]
            self.__verify_ssh(mgmt_ip=fip, user=user, retry=ssh_retries)

            sliver.label_allocations.instance = instance_props.get(AmConstants.SERVER_INSTANCE_NAME, None)

            # Attach any attached PCI Devices
            if sliver.attached_components_info is not None:
                for component in sliver.attached_components_info.devices.values():
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             host=worker_node, instance_name=sliver.label_allocations.instance,
                                             device_name=unit_id, component=component, vm_name=vmname,
                                             project_id=project_id, raise_exception=True)
            sliver.management_ip = fip
            self.__configure_components(sliver=sliver)

        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver, unit_id=unit_id, project_id=project_id)
                unit.get_sliver().label_allocations.instance = None

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

            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            unit_id = str(unit.get_reservation_id())
            unit_properties = unit.get_properties()
            project_id = unit_properties.get(Constants.PROJECT_ID, None)
            self.__cleanup(sliver=sliver, unit_id=unit_id, project_id=project_id)
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

    def __configure_component(self, *, component: ComponentSliver, user: str, mgmt_ip: str):
        try:
            if component.get_type() not in [ComponentType.SharedNIC, ComponentType.SmartNIC, ComponentType.Storage]:
                return
            if component.get_model() == Constants.OPENSTACK_VNIC_MODEL:
                return
            if component.get_type() == ComponentType.Storage:
                self.__mount_storage(component=component, mgmt_ip=mgmt_ip, user=user)
            else:
                self.configure_nic(component=component, mgmt_ip=mgmt_ip, user=user)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

    def __configure_components(self, sliver: NodeSliver):
        try:
            if sliver.attached_components_info is not None:
                for component in sliver.attached_components_info.devices.values():
                    user = self.__get_default_user(image=sliver.get_image_ref())
                    self.__configure_component(component=component,
                                               mgmt_ip=sliver.management_ip,
                                               user=user)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

    def modify(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Modify a provisioned Unit
        :param unit: unit representing VM
        :return: tuple of result status and the unit
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Modify invoked for unit: {unit}")
            current_sliver = unit.get_sliver()
            modified_sliver = unit.get_modified()

            if not isinstance(current_sliver, NodeSliver) or not isinstance(modified_sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(current_sliver)}  {type(modified_sliver)}")

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            project_id = unit.get_properties().get(Constants.PROJECT_ID, None)

            diff = current_sliver.diff(other_sliver=modified_sliver)
            if diff is not None:
                # Modify topology
                for x in diff.added.components:
                    component = modified_sliver.attached_components_info.devices[x]
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             host=current_sliver.label_allocations.instance_parent,
                                             instance_name=current_sliver.label_allocations.instance,
                                             device_name=str(unit.get_reservation_id()),
                                             component=component, vm_name=current_sliver.get_name(),
                                             project_id=project_id)

                    user = self.__get_default_user(image=current_sliver.get_image_ref())
                    self.__configure_component(component=component,
                                               mgmt_ip=current_sliver.management_ip,
                                               user=user)

                for x in diff.removed.components:
                    component = modified_sliver.attached_components_info.devices[x]
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             host=current_sliver.label_allocations.instance_parent,
                                             instance_name=current_sliver.label_allocations.instance,
                                             device_name=str(unit.get_reservation_id()),
                                             component=component, vm_name=current_sliver.get_name(),
                                             project_id=project_id, attach=False)
            else:
                # Modify configuration
                self.__configure_components(sliver=modified_sliver)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
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
        vm_name_combined = f"{unit_id}-{vm_name}"

        avail_zone = f"nova:{worker_node}"

        default_user = self.__get_default_user(image=image)

        if init_script is None:
            init_script = ""

        extra_vars = {
            AmConstants.VM_PROV_OP: AmConstants.PROV_OP_CREATE,
            AmConstants.EC2_AVAILABILITY_ZONE: avail_zone,
            AmConstants.VM_NAME: vm_name_combined,
            AmConstants.FLAVOR: flavor,
            AmConstants.IMAGE: image,
            AmConstants.HOSTNAME: vm_name,
            AmConstants.SSH_KEY: ssh_key,
            AmConstants.DEFAULT_USER: default_user,
            AmConstants.INIT_SCRIPT: init_script
        }
        ok = self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars)

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
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.PROV_OP_DELETE,
                      AmConstants.VM_NAME: vm_name}

        self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path,
                               extra_vars=extra_vars)
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

            extra_vars = {AmConstants.VM_PROV_OP: AmConstants.VM_PROV_OP_ATTACH_FIP,
                          AmConstants.VM_NAME: vmname}

            ok = self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path,
                                        extra_vars=extra_vars)

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

            self.get_logger().info(f"Returning FIP {result} for {unit_id}-{vm_name}")
            return str(result)
        finally:
            self.process_lock.release()

    def __mount_storage(self, *, component: ComponentSliver, mgmt_ip: str, user: str):
        if component.get_type() != ComponentType.Storage or not component.flags.auto_mount:
            return

        self.get_logger().debug("__mount_storage IN")
        try:
            # Grab the playbook location
            playbook_location = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]

            # Grab the playbook name
            resource_type = str(component.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CONFIG][resource_type]

            # Construct the playbook path
            playbook_path = f"{playbook_location}/{playbook}"

            # Set the variables
            extra_vars = {AmConstants.VOL_PROV_OP: AmConstants.PROV_OP_MOUNT,
                          AmConstants.VOL_NAME: component.label_allocations.local_name,
                          AmConstants.PROV_DEVICE: component.label_allocations.device_name,
                          AmConstants.VM_NAME: mgmt_ip}

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]
            self.get_logger().info(f"Executing playbook {playbook_path} to mount volume: "
                                   f"{component.label_allocations.device_name} "
                                   f"on: {mgmt_ip}")
            self.__execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                   sources=f"{mgmt_ip},", private_key_file=admin_ssh_key)

        except Exception as e:
            self.get_logger().error(f"Failed to mount the volume, we ignore the failure: {e}")
        finally:
            self.get_logger().debug("__mount_storage OUT")

    def __attach_detach_storage(self, *, playbook_path: str, inventory_path: str, vm_name: str,
                                unit_id: str, component: ComponentSliver, project_id: str, attach: bool = True):
        self.get_logger().debug("__attach_detach_storage IN")
        try:
            extra_vars = {
                AmConstants.VOL_PROV_OP: AmConstants.PROV_DETACH,
                AmConstants.VM_NAME: f"{unit_id}-{vm_name}",
                AmConstants.VOL_NAME: f"{component.get_label_allocations().local_name}",
                Constants.PROJECT_ID: f"{project_id}"
            }
            if attach:
                extra_vars[AmConstants.VOL_PROV_OP] = AmConstants.PROV_ATTACH

            ok = self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars)
            attachments = ok.get(AmConstants.ATTACHMENTS, None)
            if attachments is not None:
                for a in attachments:
                    self.get_logger().info(f"Storage volume: {component.get_name()} for project: {project_id} attached "
                                           f"as device: {a.get(AmConstants.PROV_DEVICE)}")
                    component.label_allocations.device_name = str(a.get(AmConstants.PROV_DEVICE))
        finally:
            self.get_logger().debug("__attach_detach_storage OUT")

    def __cleanup_vnic(self, *, inventory_path: str, vm_name: str, component: ComponentSliver, device_name: str):
        """
        Delete the Port for the vNIC associated with the VM
        """
        pb_location = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
        resource_type = f"{str(component.get_type())}-{component.get_model()}"
        port_pb = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
        playbook_path = f"{pb_location}/{port_pb}"

        ifs_name = None
        for ns in component.network_service_info.network_services.values():
            if ns.interface_info is None or ns.interface_info.interfaces is None:
                continue

            for ifs in ns.interface_info.interfaces.values():
                ifs_name = ifs.get_name()

        extra_vars = {AmConstants.PORT_PROV_OP: AmConstants.PROV_DETACH,
                      AmConstants.VM_NAME: f'{device_name}-{vm_name}',
                      AmConstants.PORT_NAME: f'{device_name}-{vm_name}-{vm_name}-{ifs_name}'}

        return self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars)

    def __attach_detach_pci(self, *, playbook_path: str, inventory_path: str, host: str, instance_name: str,
                            device_name: str, component: ComponentSliver, vm_name: str, project_id: str,
                            attach: bool = True, raise_exception: bool = False):
        """
        Invoke ansible playbook to attach/detach a PCI device to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param instance_name: Instance Name
        :param device_name: Device Name
        :param component: Component Sliver
        :param vm_name: VM Name
        :param project_id: Project Id
        :param attach: True for attach and False for detach
        :return:
        """
        self.get_logger().debug("__attach_detach_pci IN")
        try:
            resource_type = str(component.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            full_playbook_path = f"{playbook_path}/{playbook}"

            if component.get_type() == ComponentType.Storage:
                self.__attach_detach_storage(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                             vm_name=vm_name, unit_id=device_name, component=component,
                                             project_id=project_id, attach=attach)
                return
            mac = None
            if component.get_type() == ComponentType.SharedNIC:
                if component.get_model() == Constants.OPENSTACK_VNIC_MODEL:
                    if not attach:
                        self.__cleanup_vnic(inventory_path=inventory_path, vm_name=vm_name, device_name=device_name,
                                            component=component)
                    return
                for ns in component.network_service_info.network_services.values():
                    if ns.interface_info is None or ns.interface_info.interfaces is None:
                        continue

                    for ifs in ns.interface_info.interfaces.values():
                        mac = ifs.label_allocations.mac
            if isinstance(component.label_allocations.bdf, str):
                pci_device_list = [component.label_allocations.bdf]
            else:
                pci_device_list = component.label_allocations.bdf

            worker_node = host

            extra_vars = {AmConstants.WORKER_NODE_NAME: worker_node,
                          AmConstants.PROV_DEVICE: device_name}
            if attach:
                extra_vars[AmConstants.PCI_OPERATION] = AmConstants.PROV_ATTACH
            else:
                extra_vars[AmConstants.PCI_OPERATION] = AmConstants.PROV_DETACH

            self.get_logger().info(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            for device in pci_device_list:
                device_char_arr = self.__extract_device_addr_octets(device_address=device)
                host_vars = {
                    AmConstants.KVM_GUEST_NAME: instance_name,
                    AmConstants.PCI_DOMAIN: device_char_arr[0],
                    AmConstants.PCI_BUS: device_char_arr[1],
                    AmConstants.PCI_SLOT: device_char_arr[2],
                    AmConstants.PCI_FUNCTION: device_char_arr[3],
                }

                if mac is not None:
                    host_vars[AmConstants.MAC] = mac

                self.__execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                       extra_vars=extra_vars, host=worker_node, host_vars=host_vars)
        except Exception as e:
            self.get_logger().error(f"Error occurred attach:{attach}/detach: {not attach} device: {component}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e
        finally:
            self.get_logger().debug("__attach_detach_pci OUT")

    def __cleanup_pci(self, *, playbook_path: str, inventory_path: str, host: str, component: ComponentSliver,
                      raise_exception: bool = False):
        """
        Invoke ansible playbook to cleanup a PCI device
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param component: Component Sliver
        :return:
        """
        self.get_logger().debug("__cleanup_pci IN")
        try:
            resource_type = str(component.get_type())
            cleanup_playbooks = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CLEANUP]
            playbook = cleanup_playbooks.get(resource_type)
            if playbook is None:
                self.logger.info(f"No cleanup required for {resource_type}")
                return

            if playbook is None or inventory_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            full_playbook_path = f"{playbook_path}/{playbook}"

            worker_node = host

            extra_vars = {AmConstants.WORKER_NODE_NAME: worker_node}
            if isinstance(component.label_allocations.bdf, str):
                pci_device_list = [component.label_allocations.bdf]
            else:
                pci_device_list = component.label_allocations.bdf

            self.get_logger().info(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            for device in pci_device_list:
                extra_vars[AmConstants.PROV_DEVICE] = device
                self.__execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                       extra_vars=extra_vars)
        except Exception as e:
            self.get_logger().error(f"Error occurred cleaning device: {component}")
            if raise_exception:
                raise e
        finally:
            self.get_logger().debug("__cleanup_pci OUT")

    def __cleanup(self, *, sliver: NodeSliver, unit_id: str, project_id: str, raise_exception: bool = False):
        """
        Cleanup VM and detach PCI devices
        :param sliver: Sliver
        :param unit_id: Unit Id
        :param project_id: Project Id
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
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             instance_name=instance_name, host=worker_node,
                                             device_name=unit_id,
                                             component=device,
                                             vm_name=sliver.get_name(),
                                             project_id=project_id,
                                             attach=False, raise_exception=raise_exception)

                    self.__cleanup_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                       host=worker_node, component=device, raise_exception=raise_exception)

            resource_type = str(sliver.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            full_playbook_path = f"{playbook_path}/{playbook}"

            if sliver.get_name() is None:
                raise VmHandlerException(f"Missing required parameters vm_name: {sliver.get_name()}")

            # Delete VM
            self.__delete_vm(playbook_path=full_playbook_path, inventory_path=inventory_path,
                             vm_name=sliver.get_name(), unit_id=unit_id)
        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup {unit_id} error: {e}")
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
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.VM_PROV_OP: AmConstants.PROV_OP_GET,
                      AmConstants.VM_NAME: vm_name}
        return self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars)

    def __get_default_user(self, *, image: str) -> str:
        """
        Return default SSH user name
        :return default ssh user name
        """
        images = self.get_config()[AmConstants.RUNTIME_SECTION][AmConstants.RT_IMAGES]
        if image in images:
            return images[image]
        return AmConstants.ROOT_USER

    def configure_nic(self, *, component: ComponentSliver, mgmt_ip: str, user: str):
        """
        Configure Interfaces associated with SharedNIC or SmartNIC
        :param component Component Sliver
        :param mgmt_ip Management IP
        :param user Default Linux user for the VM
        """
        # Only do this for SharedNIC and SmartNIC
        if component.get_type() != ComponentType.SharedNIC and component.get_type() != ComponentType.SmartNIC:
            return

        if component.network_service_info is None or component.network_service_info.network_services is None:
            return

        for ns in component.network_service_info.network_services.values():
            if ns.interface_info is None or ns.interface_info.interfaces is None:
                continue
            installed_nw_mgr = False
            for ifs in ns.interface_info.interfaces.values():
                if ifs.flags is None or not ifs.flags.auto_config:
                    continue
                if not installed_nw_mgr:
                    self.__post_boot_config(mgmt_ip=mgmt_ip, user=user)
                    installed_nw_mgr = True
                self.get_logger().info(f"Configuring Interface  {ifs}")
                self.configure_network_interface(mgmt_ip=mgmt_ip, user=user, resource_type=component.get_type().name,
                                                 ipv4_address=ifs.label_allocations.ipv4,
                                                 ipv4_subnet=ifs.label_allocations.ipv4_subnet,
                                                 ipv6_address=ifs.label_allocations.ipv6,
                                                 ipv6_subnet=ifs.label_allocations.ipv6_subnet,
                                                 mac_address=ifs.label_allocations.mac,
                                                 vlan=ifs.label_allocations.vlan)

    def configure_network_interface(self, *, mgmt_ip: str, user: str, resource_type: str, mac_address: str,
                                    ipv4_address: str = None, ipv4_subnet: str = None, ipv6_address: str = None,
                                    ipv6_subnet: str = None, vlan: str = None):
        """
        Configure Network Interface inside the VM
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param resource_type Type of NIC card (SharedNIC or SmartNIC)
        :param mac_address Mac address used to identify the interface
        :param ipv4_address IPV4 address to assign
        :param ipv4_subnet IPV4 subnet to assign
        :param ipv6_address IPV6 address to assign
        :param ipv6_subnet IPV6 subnet to assign
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

            # Set the variables
            extra_vars = {AmConstants.VM_NAME: mgmt_ip,
                          AmConstants.MAC: mac_address.lower(),
                          AmConstants.IMAGE: user}

            ipv4_prefix = ipv4_subnet.split("/")[1] if ipv4_subnet is not None else "24"
            ipv6_prefix = ipv6_subnet.split("/")[1] if ipv6_subnet is not None else "64"
            if ipv4_address is not None:
                extra_vars[AmConstants.IPV4_ADDRESS] = ipv4_address
                extra_vars[AmConstants.IPV4_PREFIX] = ipv4_prefix
            if ipv6_address is not None:
                extra_vars[AmConstants.IPV6_ADDRESS] = ipv6_address
                extra_vars[AmConstants.IPV6_PREFIX] = ipv6_prefix
            if vlan is not None and resource_type != ComponentType.SharedNIC.name:
                extra_vars[AmConstants.VLAN] = vlan

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]

            self.__execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                   sources=f"{mgmt_ip},", private_key_file=admin_ssh_key, user=user)
        except Exception as e:
            self.get_logger().error(f"Exception : {e}")
            self.get_logger().error(traceback.format_exc())

    def __post_boot_config(self, *, mgmt_ip: str, user: str):
        """
        Perform post boot configuration, install required software
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        """
        try:

            # Grab the playbook location
            playbook_location = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]

            # Grab the playbook name
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CONFIG][AmConstants.PB_POST_BOOT]

            # Construct the playbook path
            playbook_path = f"{playbook_location}/{playbook}"

            # Set the variables
            extra_vars = {AmConstants.VM_NAME: mgmt_ip,
                          AmConstants.IMAGE: user}

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]

            self.__execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                   sources=f"{mgmt_ip},", private_key_file=admin_ssh_key, user=user)
        except Exception as e:
            self.get_logger().error(f"Exception : {e}")
            self.get_logger().error(traceback.format_exc())

    def __execute_command(self, *, mgmt_ip: str, user: str, command: str, timeout: int = 60, retry: int = 3):
        """
        Execute a command on the VM
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param command Command to execute
        :param timeout Timeout in seconds
        :param retry Number of retries
        :return:
        """
        for i in range(retry):
            try:
                # Construct the SSH client
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                key_file = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]
                pkey = paramiko.RSAKey.from_private_key_file(key_file)
                ssh.connect(mgmt_ip, username=user, timeout=timeout, pkey=pkey)

                # Execute the command
                stdin, stdout, stderr = ssh.exec_command(command)
                output = stdout.readlines()
                ssh.close()
                return output
            except Exception as e:
                self.get_logger().error(f"Exception : {e}")
                self.get_logger().error(traceback.format_exc())
                if i < retry - 1:
                    time.sleep(timeout)
                    self.get_logger().info(f"Retrying command {command} on VM {mgmt_ip}")
                else:
                    self.get_logger().error(f"Failed to execute command {command} on VM {mgmt_ip}")
                    raise e

    def __verify_ssh(self, *, mgmt_ip: str, user: str, timeout: int = 60, retry: int = 10):
        """
        Verify that the VM is accessible via SSH
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param retry Number of retries
        :param retry_interval Timeout in seconds

        """
        command = f"echo test ssh from {mgmt_ip} > /tmp/fabric_execute_script.sh; " \
                  f"chmod +x /tmp/fabric_execute_script.sh; /tmp/fabric_execute_script.sh"

        try:
            output = self.__execute_command(mgmt_ip=mgmt_ip, user=user, command=command,
                                            timeout=timeout, retry=retry)
            self.get_logger().info(f"Output: {output}")
        except Exception as e:
            pass

    def __execute_ansible(self, *, inventory_path: str, playbook_path: str, extra_vars: dict, sources: str = None,
                          private_key_file: str = None, host_vars: dict = None, host: str = None, user: str = None):
        ansible_python_interpreter = None
        # Head node or Worker
        if inventory_path is not None:
            ansible_python_interpreter = self.get_ansible_python_interpreter()
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=self.get_logger(),
                                       ansible_python_interpreter=ansible_python_interpreter,
                                       sources=sources)

        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        if host is not None and host_vars is not None and len(host_vars) > 0:
            for key, value in host_vars.items():
                ansible_helper.add_vars(host=host, var_name=key, value=value)

        self.get_logger().info(f"Executing playbook {playbook_path} extra_vars: {extra_vars} host_vars: {host_vars}")
        ansible_helper.run_playbook(playbook_path=playbook_path, private_key_file=private_key_file, user=user)
        return ansible_helper.get_result_callback().get_json_result_ok()
