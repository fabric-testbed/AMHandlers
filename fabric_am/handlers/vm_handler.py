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
from typing import Tuple, List, Dict

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.attached_components import ComponentSliver, ComponentType
from fim.slivers.network_node import NodeSliver
from fim.user import InstanceCatalog, CapacityHints
from jinja2 import Environment

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.utils import Utils


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

    @staticmethod
    def convert_to_string(unsafe_text_variable):
        if unsafe_text_variable is None:
            return unsafe_text_variable

        # Create a Jinja2 environment
        jinja_env = Environment()

        # Use the |string filter to convert AnsibleUnsafeText to string
        return jinja_env.from_string("{{ unsafe_text_variable | string }}").render(
            unsafe_text_variable=unsafe_text_variable)

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
            extra_vars = {AmConstants.OPERATION: AmConstants.OP_DELETE_ALL}
            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=cleanup_playbook,
                                  extra_vars=extra_vars, logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Failure to clean up existing VMs: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().debug("Clean restart - end")

        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CLEAN_RESTART,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        return result

    def __adjust_flavor(self, sliver: NodeSliver):
        runtime_section = self.get_config().get(AmConstants.RUNTIME_SECTION)
        if runtime_section is None:
            return sliver

        max_flavor = runtime_section.get(AmConstants.MAX_FLAVOR)
        if max_flavor is None:
            return sliver

        catalog = InstanceCatalog()
        max_capacities = catalog.get_instance_capacities(instance_type=max_flavor)
        allocated_capacities = sliver.get_capacity_allocations()

        if allocated_capacities.core > max_capacities.core or \
            allocated_capacities.ram > max_capacities.ram or \
                allocated_capacities.disk > max_capacities.disk:
            sliver.set_capacity_allocations(cap=max_capacities)
            sliver.set_capacity_hints(caphint=CapacityHints(instance_type=max_flavor))
            self.get_logger().info(f"Flavor for {sliver.get_name()} updated to {max_flavor} "
                                   f"capacity_hints: {sliver.get_capacity_hints()} "
                                   f"capacity_allocations: {sliver.get_capacity_allocations()}")

        return sliver

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

            self.__adjust_flavor(sliver=sliver)
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

            if disable_fip or (sliver.flags is not None and not sliver.flags.ipv4_management):
                self.get_logger().info("Floating IP is disabled, using IPV6 Global Unicast Address")
                fip = instance_props.get(AmConstants.SERVER_ACCESS_IPV6, None)
            else:
                # Attach FIP
                fip = self.__attach_fip(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                        vm_name=vmname, unit_id=unit_id)

            # Verify SSH connectivity
            ssh_retries = self.get_config()[AmConstants.RUNTIME_SECTION][AmConstants.RT_SSH_RETRIES]
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]
            Utils.verify_ssh(mgmt_ip=fip, user=user, retry=ssh_retries, ssh_key_file=admin_ssh_key,
                             logger=self.get_logger())

            sliver.label_allocations.instance = instance_props.get(AmConstants.SERVER_INSTANCE_NAME, None)

            # Attach any attached PCI Devices
            if sliver.attached_components_info is not None:
                for component in sliver.attached_components_info.devices.values():
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             host=worker_node, instance_name=sliver.label_allocations.instance,
                                             device_name=unit_id, component=component, vm_name=vmname,
                                             project_id=project_id, raise_exception=True, mgmt_ip=fip, user=user)

            # REBOOT the VM
            self.__perform_os_server_action(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                            vm_name=vmname, unit_id=unit_id, action=AmConstants.OP_REBOOT)

            Utils.verify_ssh(mgmt_ip=fip, user=user, retry=ssh_retries, ssh_key_file=admin_ssh_key,
                             logger=self.get_logger())

            if fip:
                sliver.management_ip = str(fip)
            # Configure Components - only gets triggered via Portal for now
            self.__configure_components(sliver=sliver)

        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
            # Delete VM in case of failure
            if sliver is not None and unit_id is not None:
                time.sleep(5)
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

    def poa(self, unit: ConfigToken, data: dict) -> Tuple[dict, ConfigToken]:
        """
        POA - perform operational action on a VM
        """
        try:
            self.get_logger().info(f"POA invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            operation = data.get("operation")

            if operation == AmConstants.OP_CPUINFO:
                result = self.__poa_cpuinfo(unit=unit, data=data)
            elif operation == AmConstants.OP_NUMAINFO:
                result = self.__poa_numainfo(unit=unit, data=data)
            elif operation == AmConstants.OP_CPUPIN:
                result = self.__poa_cpupin(unit=unit, data=data)
            elif operation == AmConstants.OP_NUMATUNE:
                result = self.__poa_numatune(unit=unit, data=data)
            elif operation == AmConstants.OP_REBOOT:
                result = self.__poa_reboot(unit=unit, data=data)
            elif operation == AmConstants.OP_ADDKEY or operation == AmConstants.OP_REMOVEKEY:
                result = self.__poa_sshkey(unit=unit, data=data, operation=operation)
            else:
                raise VmHandlerException(f"Unsupported {operation}")

        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
        finally:

            self.get_logger().info(f"POA completed")
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
                    user = self.__get_default_user(image=current_sliver.get_image_ref())
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             host=current_sliver.label_allocations.instance_parent,
                                             instance_name=current_sliver.label_allocations.instance,
                                             device_name=str(unit.get_reservation_id()),
                                             component=x, vm_name=current_sliver.get_name(),
                                             project_id=project_id, mgmt_ip=current_sliver.get_management_ip(),
                                             user=user)

                    user = self.__get_default_user(image=current_sliver.get_image_ref())
                    self.__configure_component(component=x,
                                               mgmt_ip=current_sliver.management_ip,
                                               user=user)

                for x in diff.removed.components:
                    self.__attach_detach_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                             host=current_sliver.label_allocations.instance_parent,
                                             instance_name=current_sliver.label_allocations.instance,
                                             device_name=str(unit.get_reservation_id()),
                                             component=x, vm_name=current_sliver.get_name(),
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

    @staticmethod
    def __build_user_data(*, default_user: str, ssh_key: str, init_script: str = None):
        user_data = "#!/bin/bash\n"
        ssh_keys = ssh_key.split(",")
        for key in ssh_keys:
            user_data += f"echo {key} >> /home/{default_user}/.ssh/authorized_keys\n"

        if init_script is not None:
            user_data += init_script

        return user_data

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

        user_data = self.__build_user_data(default_user=default_user, ssh_key=ssh_key, init_script=init_script)

        extra_vars = {
            AmConstants.OPERATION: AmConstants.OP_CREATE,
            AmConstants.EC2_AVAILABILITY_ZONE: avail_zone,
            AmConstants.VM_NAME: vm_name_combined,
            AmConstants.FLAVOR: flavor,
            AmConstants.IMAGE: image,
            AmConstants.HOSTNAME: vm_name,
            AmConstants.SSH_KEY: ssh_key,
            AmConstants.DEFAULT_USER: default_user,
            AmConstants.INIT_SCRIPT: init_script,
            AmConstants.USER_DATA: user_data
        }
        ok = Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars,
                                   logger=self.get_logger())

        server = ok.get(AmConstants.SERVER, None)
        # Added this code for enabling test suite
        if server is None:
            server = str(ok[AmConstants.ANSIBLE_FACTS][AmConstants.SERVER])
            server = json.loads(server)

        result = {
            AmConstants.SERVER_VM_STATE: self.convert_to_string(server[AmConstants.SERVER_VM_STATE]),
            AmConstants.SERVER_INSTANCE_NAME: self.convert_to_string(server[AmConstants.SERVER_INSTANCE_NAME]),
            AmConstants.SERVER_ACCESS_IPV4: self.convert_to_string(server[AmConstants.SERVER_ACCESS_IPV4])
        }
        if server[AmConstants.SERVER_ACCESS_IPV6] is not None:
            result[AmConstants.SERVER_ACCESS_IPV6] = self.convert_to_string(server[AmConstants.SERVER_ACCESS_IPV6])
        self.get_logger().debug(f"Returning properties {result}")

        return result

    def __delete_vm(self, *, playbook_path: str, inventory_path: str, sliver: NodeSliver, unit_id: str) -> bool:
        """
        Invoke ansible playbook to remove a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param sliver: VM Sliver
        :param unit_id: Unit Id
        :return: True or False representing success/failure
        """
        vm_name = f"{unit_id}-{sliver.get_name()}"

        extra_vars = {AmConstants.OPERATION: AmConstants.OP_DELETE,
                      AmConstants.VM_NAME: vm_name}

        # Retry Delete VM configured number of times in case of failure to delete VMs
        # Handle Openstack Delete VM API timeout/errors
        delete_retries = self.get_config().get(AmConstants.RUNTIME_SECTION).get(AmConstants.RT_DELETE_RETRIES, 3)
        for i in range(delete_retries):
            try:
                self.get_logger().debug(f"Delete attempt # {i}")
                Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path,
                                      extra_vars=extra_vars, logger=self.get_logger())
                time.sleep(5)
                break
            except Exception as e:
                if i < delete_retries:
                    continue
                else:
                    self.get_logger().warning(f'Delete Failed - cleanup attempts {delete_retries}')
                    raise e

        playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
        playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.OPERATION][AmConstants.OP_DELETE]
        full_playbook_path = f"{playbook_path}/{playbook}"

        # Verify VM has been deleted, if not attempt to do cleanup via libvirt
        extra_vars = {
            AmConstants.WORKER_NODE_NAME: sliver.get_label_allocations().instance_parent,
            AmConstants.OPERATION: AmConstants.OP_DELETE,
            AmConstants.KVM_GUEST_NAME: sliver.get_label_allocations().instance
        }
        for i in range(delete_retries):
            try:
                self.get_logger().debug(f"Delete via libvirt attempt # {i}")
                Utils.execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                      extra_vars=extra_vars, logger=self.get_logger())
                time.sleep(5)
                break
            except Exception as e:
                if i < delete_retries:
                    continue
                else:
                    self.get_logger().warning(f'Delete Failed - cleanup attempts {delete_retries}')
                    self.get_logger().error(e)

        extra_vars = {
            AmConstants.WORKER_NODE_NAME: sliver.get_label_allocations().instance_parent,
            AmConstants.OPERATION: AmConstants.OP_IS_DELETED,
            AmConstants.KVM_GUEST_NAME: sliver.get_label_allocations().instance
        }
        try:
            Utils.execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                  extra_vars=extra_vars, logger=self.get_logger())
        except Exception as e:
            self.get_logger().warning(f'List VM post deletion failed')
            self.get_logger().error(e)

        return True

    def __attach_fip(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str) -> str:
        """
        Invoke ansible playbook to attach a floating IP to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :return: floating ip assigned to the VM
        """
        try:
            self.process_lock.acquire()
            vmname = f"{unit_id}-{vm_name}"

            extra_vars = {AmConstants.OPERATION: AmConstants.OP_ATTACH_FIP,
                          AmConstants.VM_NAME: vmname}

            ok = Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path,
                                       extra_vars=extra_vars, logger=self.get_logger())

            if self.test_mode:
                floating_ip = ok[AmConstants.ANSIBLE_FACTS][AmConstants.FLOATING_IP]
                floating_ip = json.loads(floating_ip)
                return self.convert_to_string(floating_ip[AmConstants.FLOATING_IP_ADDRESS])

            floating_ip = ok[AmConstants.FLOATING_IP]
            result = None
            if floating_ip is None:
                self.get_logger().info("Floating IP returned by attach was null, trying to get via get_vm")
                ok = self.__perform_os_server_action(playbook_path=playbook_path, inventory_path=inventory_path,
                                                     vm_name=vm_name, unit_id=unit_id, action=AmConstants.OP_GET)
                self.get_logger().info(f"Info returned by GET VM: {ok}")
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
            extra_vars = {AmConstants.OPERATION: AmConstants.OP_MOUNT,
                          AmConstants.VOL_NAME: component.label_allocations.local_name,
                          AmConstants.DEVICE: component.label_allocations.device_name,
                          AmConstants.VM_NAME: mgmt_ip}

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]
            self.get_logger().info(f"Executing playbook {playbook_path} to mount volume: "
                                   f"{component.label_allocations.device_name} "
                                   f"on: {mgmt_ip}")
            Utils.execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                  sources=f"{mgmt_ip},", private_key_file=admin_ssh_key, logger=self.get_logger())

        except Exception as e:
            self.get_logger().error(f"Failed to mount the volume, we ignore the failure: {e}")
        finally:
            self.get_logger().debug("__mount_storage OUT")

    def __attach_detach_storage(self, *, playbook_path: str, inventory_path: str, vm_name: str,
                                unit_id: str, component: ComponentSliver, project_id: str, attach: bool = True):
        self.get_logger().debug("__attach_detach_storage IN")
        try:
            extra_vars = {
                AmConstants.OPERATION: AmConstants.OP_DETACH,
                AmConstants.VM_NAME: f"{unit_id}-{vm_name}",
                AmConstants.VOL_NAME: f"{component.get_label_allocations().local_name}",
                Constants.PROJECT_ID: f"{project_id}"
            }
            if attach:
                extra_vars[AmConstants.OPERATION] = AmConstants.OP_ATTACH

            ok = Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path,
                                       extra_vars=extra_vars, logger=self.get_logger())
            attachments = ok.get(AmConstants.ATTACHMENTS, None)
            if attachments is not None:
                for a in attachments:
                    self.get_logger().info(f"Storage volume: {component.get_name()} for project: {project_id} attached "
                                           f"as device: {a.get(AmConstants.DEVICE)}")
                    component.label_allocations.device_name = self.convert_to_string(a.get(AmConstants.DEVICE))
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

        extra_vars = {AmConstants.OPERATION: AmConstants.OP_DETACH,
                      AmConstants.VM_NAME: f'{device_name}-{vm_name}',
                      AmConstants.PORT_NAME: f'{device_name}-{vm_name}-{vm_name}-{ifs_name}'}

        return Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars,
                                     logger=self.get_logger())

    def __attach_detach_multiple_function_pci(self, *, playbook_path: str, inventory_path: str, host: str,
                                              instance_name: str, device_name: str, component: ComponentSliver,
                                              vm_name: str, project_id: str, attach: bool = True,
                                              raise_exception: bool = False, mgmt_ip: str = None, user: str = None):
        """
        Invoke ansible playbook to attach/detach a PCI device with multiple functions to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param host: host
        :param instance_name: Instance Name
        :param device_name: Device Name
        :param component: Component Sliver
        :param vm_name: VM Name
        :param project_id: Project Id
        :param attach: True for attach and False for detach
        :param mgmt_ip Management IP
        :param user default user
        :return:
        """
        self.get_logger().debug("__attach_detach_multiple_function_pci IN")
        try:
            resource_type = str(component.get_type())
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            full_playbook_path = f"{playbook_path}/{playbook}"

            # Grab the Mac addresses
            mac = []
            if component.get_type() in [ComponentType.SmartNIC, ComponentType.SharedNIC]:
                ns_name = list(component.network_service_info.network_services.keys())[0]
                ns = component.network_service_info.network_services[ns_name]
                interface_names = list(ns.interface_info.interfaces.keys())
                if len(interface_names) > 0:
                    for ifc in interface_names:
                        mac.append(ns.interface_info.interfaces[ifc].label_allocations.mac.lower())

            if isinstance(component.labels.bdf, str):
                pci_device_list = [component.labels.bdf]
            else:
                pci_device_list = component.labels.bdf

            self.get_logger().info(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            bdf = str(pci_device_list[0])
            #pattern = r'\d+:([\da-fA-F]+):\d+\.\d'
            #matches = re.match(pattern, bdf)
            matches = re.split("(.*):(.*):(.*)\\.(.*)", bdf)

            self.get_logger().info(f"Matches: {matches}")

            extra_vars = {
                AmConstants.WORKER_NODE_NAME: host,
                AmConstants.NUM_PCI: len(pci_device_list),
                AmConstants.MAC: mac,
                AmConstants.PCI_BDF: bdf

            }

            # For SN1022; USB tags not required; needed for CIEN rack
            if "SN1022" in component.get_model():
                extra_vars[AmConstants.USB_REQUIRED] = "no"

            if attach:
                extra_vars[AmConstants.OPERATION] = AmConstants.OP_ATTACH
            else:
                extra_vars[AmConstants.OPERATION] = AmConstants.OP_DETACH

            host_vars = {
                AmConstants.KVM_GUEST_NAME: instance_name,
                AmConstants.PCI_DOMAIN: f"0x{matches[1]}",
                AmConstants.PCI_BUS: f"0x{matches[2]}",
                AmConstants.PCI_SLOT: f"0x{matches[3]}"
            }
            ok = Utils.execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                       extra_vars=extra_vars, host=host, host_vars=host_vars, logger=self.get_logger())

            # In case of Attach, determine the PCI device id from inside the VM
            # Also, determine the ethernet interface name in case of Shared/Smart NIC
            # This is done in a separate loop on purpose to give VM OS to identify PCI devices
            # and associate mac addresses with them
            if attach:
                ansible_facts = ok.get(AmConstants.ANSIBLE_FACTS)
                if ansible_facts:
                    pci_device_number = self.convert_to_string(ansible_facts.get(AmConstants.PCI_DEVICE_NUMBER))
                    self.__determine_pci_address_in_vm(component=component,
                                                       pci_device_number=pci_device_number,
                                                       mgmt_ip=mgmt_ip,
                                                       user=user)
        except Exception as e:
            self.get_logger().error(f"Error occurred attach:{attach}/detach: {not attach} device: {component}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e
        finally:
            self.get_logger().debug("__attach_detach_multiple_function_pci OUT")

    def __determine_pci_address_in_vm(self, *, component: ComponentSliver, mgmt_ip: str, user: str,
                                      pci_device_number: str):
        if not pci_device_number or not len(pci_device_number):
            return
        try:
            if isinstance(component.labels.bdf, str):
                pci_device_list = [component.labels.bdf]
            else:
                pci_device_list = component.labels.bdf

            ns = None
            interface_names = None
            if component.get_type() in [ComponentType.SmartNIC, ComponentType.SharedNIC]:
                ns_name = list(component.network_service_info.network_services.keys())[0]
                ns = component.network_service_info.network_services[ns_name]
                interface_names = list(ns.interface_info.interfaces.keys())

            for idx in range(len(pci_device_list)):
                mac = None
                if ns and interface_names and len(interface_names) > 0:
                    mac = ns.interface_info.interfaces[interface_names[idx]].label_allocations.mac.lower()
                ok = self.__post_boot_config(mgmt_ip=mgmt_ip, user=user, pci_device_number=pci_device_number,
                                             mac=mac)
                interface_name = None
                bdf_facts = None
                ansible_facts = ok.get(AmConstants.ANSIBLE_FACTS)
                self.logger.info(f"Ansible Facts: {ansible_facts}")
                if ansible_facts is not None:
                    combined_facts = ansible_facts.get(AmConstants.COMBINED_FACTS)
                    self.logger.info(f"Combined Facts: {combined_facts}")
                    if combined_facts is not None:
                        bdf_facts = self.convert_to_string(combined_facts.get(AmConstants.PCI_BDF))
                        interface_name = self.convert_to_string(combined_facts.get(AmConstants.INTERFACE_NAME))

                self.logger.info(f"BDF Facts: {bdf_facts} Interface Name: {interface_name}")
                if bdf_facts is not None:
                    bdf_list = str(bdf_facts).split("\n")
                    for bdf in bdf_list:
                        if bdf.endswith(":"):
                            bdf = bdf[:-1]
                        if bdf not in component.label_allocations.bdf and len(bdf):
                            component.label_allocations.bdf.append(bdf)
                if interface_name is not None:
                    ns.interface_info.interfaces[interface_names[idx]].label_allocations.local_name = str(
                        interface_name)
                self.logger.info(f"Label Allocations: {component.label_allocations} {ns}")
                if ns is not None:
                    self.logger.info(f"{ns.interface_info.interfaces.values()}")
        except Exception as e:
            self.get_logger().error(f"Error occurred get PCI information from the VM: {component}")
            self.get_logger().error(traceback.format_exc())

    def __attach_detach_pci(self, *, playbook_path: str, inventory_path: str, host: str, instance_name: str,
                            device_name: str, component: ComponentSliver, vm_name: str, project_id: str,
                            attach: bool = True, raise_exception: bool = False, mgmt_ip: str = None, user: str = None):
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
        :param mgmt_ip Management IP
        :param user default user
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

            if isinstance(component.labels.bdf, str):
                pci_device_list = [component.labels.bdf]
            else:
                pci_device_list = component.labels.bdf

            if component.get_type() == ComponentType.FPGA or \
                    (pci_device_list and len(pci_device_list) > 1 and "multi" in playbook):
                self.__attach_detach_multiple_function_pci(playbook_path=playbook_path, inventory_path=inventory_path,
                                                           host=host, instance_name=instance_name,
                                                           device_name=device_name, component=component,
                                                           vm_name=vm_name, project_id=project_id,
                                                           attach=attach, raise_exception=raise_exception,
                                                           mgmt_ip=mgmt_ip, user=user)
                return

            if component.get_type() == ComponentType.Storage:
                self.__attach_detach_storage(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                             vm_name=vm_name, unit_id=device_name, component=component,
                                             project_id=project_id, attach=attach)
                return

            if component.get_type() == ComponentType.SharedNIC:
                if component.get_model() == Constants.OPENSTACK_VNIC_MODEL:
                    if not attach:
                        self.__cleanup_vnic(inventory_path=inventory_path, vm_name=vm_name, device_name=device_name,
                                            component=component)
                    return
            # Grab the Mac addresses
            interface_names = []
            ns = None
            if component.get_type() in [ComponentType.SmartNIC, ComponentType.SharedNIC]:
                ns_name = list(component.network_service_info.network_services.keys())[0]
                ns = component.network_service_info.network_services[ns_name]
                interface_names = list(ns.interface_info.interfaces.keys())

            worker_node = host

            extra_vars = {AmConstants.WORKER_NODE_NAME: worker_node,
                          AmConstants.DEVICE: device_name}
            if attach:
                extra_vars[AmConstants.OPERATION] = AmConstants.OP_ATTACH
                component.label_allocations.bdf = []
            else:
                extra_vars[AmConstants.OPERATION] = AmConstants.OP_DETACH

            self.get_logger().info(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            idx = 0
            pci_device_number = None
            # Attach/ Detach the PCI Device
            for device in pci_device_list:
                device_char_arr = self.__extract_device_addr_octets(device_address=device)
                device = device.replace("0000:", "")
                host_vars = {
                    AmConstants.KVM_GUEST_NAME: instance_name,
                    AmConstants.PCI_DOMAIN: device_char_arr[0],
                    AmConstants.PCI_BUS: device_char_arr[1],
                    AmConstants.PCI_SLOT: device_char_arr[2],
                    AmConstants.PCI_FUNCTION: device_char_arr[3],
                    AmConstants.PCI_BDF: device
                }
                if len(interface_names) > 0:
                    mac = ns.interface_info.interfaces[interface_names[idx]].label_allocations.mac.lower()
                    host_vars[AmConstants.MAC] = mac

                ok = Utils.execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                           extra_vars=extra_vars, host=worker_node, host_vars=host_vars,
                                           logger=self.get_logger())

                if attach and ok:
                    idx += 1
                    ansible_facts = ok.get(AmConstants.ANSIBLE_FACTS)
                    if ansible_facts:
                        pci_device_number = self.convert_to_string(ansible_facts.get(AmConstants.PCI_DEVICE_NUMBER))

            # In case of Attach, determine the PCI device id from inside the VM
            # Also, determine the ethernet interface name in case of Shared/Smart NIC
            # This is done in a separate loop on purpose to give VM OS to identify PCI devices
            # and associate mac addresses with them
            if attach:
                self.__determine_pci_address_in_vm(component=component,
                                                   pci_device_number=pci_device_number,
                                                   mgmt_ip=mgmt_ip,
                                                   user=user)
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
            if isinstance(component.labels.bdf, str):
                pci_device_list = [component.labels.bdf]
            else:
                pci_device_list = component.labels.bdf

            self.get_logger().info(f"Device List Size: {len(pci_device_list)} List: {pci_device_list}")
            for device in pci_device_list:
                extra_vars[AmConstants.DEVICE] = device
                Utils.execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                      extra_vars=extra_vars, logger=self.get_logger())
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
                             sliver=sliver, unit_id=unit_id)
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

    def __perform_virsh_server_action(self, *, playbook_path: str, inventory_path: str, worker_node_name: str,
                                      instance_name: str, operation: str, vcpu_cpu_map: List[Dict[str, str]] = None,
                                      node_set: List[str] = None):
        """
        Invoke ansible playbook to perform a server action via openstack commands
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param worker_node_name: Worker Node Name
        :param instance_name: VM Name
        :param operation: Action to be performed
        :return: OK result
        """
        playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.OPERATION][operation]
        playbook_path_full = f"{playbook_path}/{playbook}"
        extra_vars = {AmConstants.OPERATION: operation,
                      AmConstants.KVM_GUEST_NAME: instance_name,
                      AmConstants.WORKER_NODE_NAME: worker_node_name}

        if vcpu_cpu_map is not None:
            extra_vars[AmConstants.VCPU_CPU_MAP] = vcpu_cpu_map

        if node_set is not None:
            extra_vars[AmConstants.NODE_SET] = node_set

        return Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path_full,
                                     extra_vars=extra_vars, logger=self.get_logger())

    def __perform_os_server_action(self, *, playbook_path: str, inventory_path: str, vm_name: str, unit_id: str,
                                   action: str):
        """
        Invoke ansible playbook to perform a server action via openstack commands
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param vm_name: VM Name
        :param unit_id: Unit Id
        :param action: Action to be performed
        :return: OK result
        """
        vm_name = f"{unit_id}-{vm_name}"

        extra_vars = {AmConstants.OPERATION: action,
                      AmConstants.VM_NAME: vm_name}
        return Utils.execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path, extra_vars=extra_vars,
                                     logger=self.get_logger())

    def __get_default_user(self, *, image: str) -> str:
        """
        Return default SSH user name
        :return default ssh user name
        """
        images = self.get_config()[AmConstants.RUNTIME_SECTION][AmConstants.RT_IMAGES]
        if image in images:
            return images[image]
        if AmConstants.ROCKY in image:
            return AmConstants.ROCKY
        if AmConstants.UBUNTU in image:
            return AmConstants.UBUNTU
        if AmConstants.CENTOS in image:
            return AmConstants.CENTOS
        if AmConstants.DEBIAN in image:
            return AmConstants.DEBIAN
        if AmConstants.FEDORA in image:
            return AmConstants.FEDORA
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
            for ifs in ns.interface_info.interfaces.values():
                if ifs.flags is None or not ifs.flags.auto_config:
                    continue
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

            Utils.execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                  sources=f"{mgmt_ip},", private_key_file=admin_ssh_key, user=user,
                                  logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Exception : {e}")
            self.get_logger().error(traceback.format_exc())

    def __post_boot_config(self, *, mgmt_ip: str, user: str, pci_device_number: str = None, mac: str=None):
        """
        Perform post boot configuration:
        - Grabs the PCI device name from inside the VM
        - For NIC cards, also gets the interface name
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param mac Mac Address in case of NICs
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

            if pci_device_number is not None:
                extra_vars[AmConstants.OPERATION] = 'get_pci'
                extra_vars[AmConstants.PCI_DEVICE_NUMBER] = pci_device_number

            if mac is not None:
                extra_vars[AmConstants.MAC] = mac

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]

            return Utils.execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                         sources=f"{mgmt_ip},", private_key_file=admin_ssh_key, user=user,
                                         logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Exception : {e}")
            self.get_logger().error(traceback.format_exc())

    def __poa_cpuinfo(self, unit: ConfigToken, data: dict) -> dict:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        try:
            self.get_logger().info(f"POA-cpuinfo started")

            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            worker_node = sliver.label_allocations.instance_parent
            vmname = f"{unit.get_reservation_id()}-{sliver.get_name()}"

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # Grab VcpuInfo for the VM and cpu information for the host
            ok = self.__perform_virsh_server_action(playbook_path=playbook_path, inventory_path=inventory_path,
                                                    worker_node_name=worker_node, operation=AmConstants.OP_CPUINFO,
                                                    instance_name=sliver.label_allocations.instance)
            ansible_facts = ok.get(AmConstants.ANSIBLE_FACTS)
            cpu_info = json.loads(self.convert_to_string(ansible_facts.get(f"{AmConstants.OP_CPUINFO}")[0]))
            self.logger.info(f"{AmConstants.OP_CPUINFO} for {vmname}: {cpu_info}")

            result[Constants.PROPERTY_POA_INFO] = {
                AmConstants.OPERATION: data.get(AmConstants.OPERATION),
                Constants.POA_ID: data.get(Constants.POA_ID),
                Constants.PROPERTY_CODE: Constants.RESULT_CODE_OK,
                Constants.PROPERTY_INFO: {
                    AmConstants.OP_CPUINFO: cpu_info
                }
            }
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e,
                      Constants.PROPERTY_POA_INFO: {
                          "operation": data.get("operation"),
                          "poa_id": data.get("poa_id"),
                          "code": Constants.RESULT_CODE_EXCEPTION
                      }
                      }
        finally:
            self.get_logger().info(f"POA-cpuinfo completed")

        return result

    def __poa_numainfo(self, unit: ConfigToken, data: dict) -> dict:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        try:
            self.get_logger().info(f"POA-numainfo started")

            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            worker_node = sliver.label_allocations.instance_parent
            vmname = f"{unit.get_reservation_id()}-{sliver.get_name()}"

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # Grab Numa Stat Info for the VM and Numa Info for the Host
            ok = self.__perform_virsh_server_action(playbook_path=playbook_path, inventory_path=inventory_path,
                                                    worker_node_name=worker_node, operation=AmConstants.OP_NUMAINFO,
                                                    instance_name=sliver.label_allocations.instance)

            ansible_facts = ok.get(AmConstants.ANSIBLE_FACTS)
            numainfo_vm = ansible_facts.get(f"{AmConstants.OP_NUMAINFO}_{AmConstants.VM}")
            numainfo_host = ansible_facts.get(f"{AmConstants.OP_NUMAINFO}_{AmConstants.HOST}")
            numainfo = {}
            if ansible_facts is not None:
                if numainfo_vm is not None:
                    numainfo_vm = Utils.parse_numastat(numastat_output=numainfo_vm)
                    numainfo[sliver.label_allocations.instance] = numainfo_vm
                if numainfo_host is not None:
                    numainfo_host = Utils.parse_numactl(numactl_output=numainfo_host)
                    numainfo[worker_node] = numainfo_host

            self.logger.info(f"{AmConstants.OP_NUMAINFO} for {vmname}: {numainfo}")
            result[Constants.PROPERTY_POA_INFO] = {
                AmConstants.OPERATION: data.get(AmConstants.OPERATION),
                Constants.POA_ID: data.get(Constants.POA_ID),
                Constants.PROPERTY_CODE: Constants.RESULT_CODE_OK,
                Constants.PROPERTY_INFO: {
                    AmConstants.OP_NUMAINFO: numainfo
                }
            }
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e,
                      Constants.PROPERTY_POA_INFO: {
                          "operation": data.get("operation"),
                          "poa_id": data.get("poa_id"),
                          "code": Constants.RESULT_CODE_EXCEPTION
                      }
                      }
        finally:
            self.get_logger().info(f"POA-numainfo completed")

        return result

    def __poa_reboot(self, unit: ConfigToken, data: dict) -> dict:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        try:
            self.get_logger().info(f"POA-reboot started")

            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            resource_type = str(sliver.get_type())
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]

            if playbook is None or inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook: {playbook} "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")
            playbook_path_full = f"{playbook_path}/{playbook}"

            # REBOOT the VM
            self.__perform_os_server_action(playbook_path=playbook_path_full, inventory_path=inventory_path,
                                            vm_name=sliver.get_name(), unit_id=str(unit.get_reservation_id()),
                                            action=AmConstants.OP_REBOOT)

            result[Constants.PROPERTY_POA_INFO] = {
                AmConstants.OPERATION: data.get(AmConstants.OPERATION),
                Constants.POA_ID: data.get(Constants.POA_ID),
                Constants.PROPERTY_CODE: Constants.RESULT_CODE_OK,
                Constants.PROPERTY_POA_INFO: {
                    "operation": data.get("operation"),
                    "poa_id": data.get("poa_id"),
                    "code": Constants.RESULT_CODE_OK
                }
            }
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e,
                      Constants.PROPERTY_POA_INFO: {
                          "operation": data.get("operation"),
                          "poa_id": data.get("poa_id"),
                          "code": Constants.RESULT_CODE_EXCEPTION
                      }
                      }
        finally:
            self.get_logger().info(f"POA-reboot completed")

        return result

    def __poa_cpupin(self, unit: ConfigToken, data: dict) -> dict:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        try:
            self.get_logger().info(f"POA-cpupin started")

            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            worker_node = sliver.label_allocations.instance_parent
            vcpu_cpu_map = data.get(AmConstants.VCPU_CPU_MAP)

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # Pin vCPU to requested CPUs
            self.__perform_virsh_server_action(playbook_path=playbook_path, inventory_path=inventory_path,
                                               worker_node_name=worker_node, operation=AmConstants.OP_CPUPIN,
                                               instance_name=sliver.label_allocations.instance,
                                               vcpu_cpu_map=vcpu_cpu_map)
            result[Constants.PROPERTY_POA_INFO] = {
                AmConstants.OPERATION: data.get(AmConstants.OPERATION),
                Constants.POA_ID: data.get(Constants.POA_ID),
                Constants.PROPERTY_CODE: Constants.RESULT_CODE_OK,
                Constants.PROPERTY_POA_INFO: {
                    "operation": data.get("operation"),
                    "poa_id": data.get("poa_id"),
                    "code": Constants.RESULT_CODE_OK
                }
            }
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
        finally:
            self.get_logger().info(f"POA-cpupin completed")

        return result

    def __poa_numatune(self, unit: ConfigToken, data: dict) -> dict:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        try:
            self.get_logger().info(f"POA-numatune started")

            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            worker_node = sliver.label_allocations.instance_parent
            node_set = data.get(AmConstants.NODE_SET)

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            if inventory_path is None or playbook_path is None:
                raise VmHandlerException(f"Missing config parameters "
                                         f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            # Numa Tune VM Guest to the requested Numa Nodes
            self.__perform_virsh_server_action(playbook_path=playbook_path, inventory_path=inventory_path,
                                               worker_node_name=worker_node, operation=AmConstants.OP_NUMATUNE,
                                               instance_name=sliver.label_allocations.instance,
                                               node_set=node_set)
            result[Constants.PROPERTY_POA_INFO] = {
                AmConstants.OPERATION: data.get(AmConstants.OPERATION),
                Constants.POA_ID: data.get(Constants.POA_ID),
                Constants.PROPERTY_CODE: Constants.RESULT_CODE_OK,
                Constants.PROPERTY_POA_INFO: {
                    "operation": data.get("operation"),
                    "poa_id": data.get("poa_id"),
                    "code": Constants.RESULT_CODE_OK
                }
            }
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e,
                      Constants.PROPERTY_POA_INFO: {
                          "operation": data.get("operation"),
                          "poa_id": data.get("poa_id"),
                          "code": Constants.RESULT_CODE_EXCEPTION
                      }
                      }
        finally:
            self.get_logger().info(f"POA-numatune completed")

        return result

    def __poa_sshkey(self, unit: ConfigToken, data: dict, operation: str) -> dict:
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}

        try:
            self.get_logger().info(f"POA-sshkey started {operation}")

            sliver = unit.get_sliver()
            if not isinstance(sliver, NodeSliver):
                raise VmHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver is None:
                raise VmHandlerException(f"Unit # {unit} has no assigned slivers")

            # Grab the playbook location
            playbook_location = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]

            # Grab the playbook name
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CONFIG][AmConstants.PB_SSH_KEYS]

            # Construct the playbook path
            playbook_path = f"{playbook_location}/{playbook}"

            if playbook_path is None:
                raise VmHandlerException(f"Missing config parameters playbook_path: {playbook_path}")

            user = self.__get_default_user(image=sliver.get_image_ref())

            # Set the variables
            extra_vars = {AmConstants.VM_NAME: sliver.management_ip,
                          AmConstants.USER: user,
                          AmConstants.KEYS: data.get(AmConstants.KEYS),
                          AmConstants.OPERATION: operation}

            # Grab the SSH Key
            admin_ssh_key = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.ADMIN_SSH_KEY]

            Utils.execute_ansible(inventory_path=None, playbook_path=playbook_path, extra_vars=extra_vars,
                                  sources=f"{sliver.management_ip},", private_key_file=admin_ssh_key, user=user,
                                  logger=self.get_logger())

            result[Constants.PROPERTY_POA_INFO] = {
                AmConstants.OPERATION: data.get(AmConstants.OPERATION),
                Constants.POA_ID: data.get(Constants.POA_ID),
                Constants.PROPERTY_CODE: Constants.RESULT_CODE_OK,
                Constants.PROPERTY_POA_INFO: {
                    "operation": data.get("operation"),
                    "poa_id": data.get("poa_id"),
                    "code": Constants.RESULT_CODE_OK
                }
            }
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())

            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_POA,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e,
                      Constants.PROPERTY_POA_INFO: {
                          "operation": data.get("operation"),
                          "poa_id": data.get("poa_id"),
                          "code": Constants.RESULT_CODE_EXCEPTION
                      }
                      }
        finally:
            self.get_logger().info(f"POA-sshkey completed {operation}")

        return result
