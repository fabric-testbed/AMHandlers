import traceback
from typing import Tuple

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.interface_info import InterfaceSliver
from fim.slivers.network_service import NetworkServiceSliver, ServiceType

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class VnicNetHandlerException(Exception):
    """
    vNIC Net Handler Exception
    """
    pass


class VnicNetHandler(HandlerBase):
    """
    vNIC Net Handler; This handler runs on the Site Head node
    """
    test_mode = False

    def get_ansible_python_interpreter(self) -> str:
        return self.get_config()[AmConstants.ANSIBLE_SECTION][
                AmConstants.ANSIBLE_PYTHON_INTERPRETER]

    def clean_restart(self):
        """
        Clean Restart
        """
        self.get_logger().debug("Clean restart - begin")
        self.get_logger().debug("Clean restart - end")

        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CLEAN_RESTART,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        return result

    def create(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Handle Create for Network Services for OpenStack vNIC
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        sliver = None
        unit_id = None
        try:
            self.get_logger().info(f"Create invoked for unit: {unit}")
            sliver = unit.get_sliver()
            unit_id = str(unit.get_reservation_id())

            if sliver is None:
                raise VnicNetHandlerException(f"Unit # {unit} has no assigned slivers")

            if not isinstance(sliver, NetworkServiceSliver):
                raise VnicNetHandlerException(f"Invalid Sliver type {type(sliver)}")

            if sliver.get_type() != ServiceType.L2Bridge:
                raise VnicNetHandlerException(f"Unsupported Service - {sliver.get_type()}")

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][str(sliver.get_type())]

            if playbook is None or inventory_path is None:
                raise VnicNetHandlerException(f"Missing config parameters playbook: {playbook} "
                                              f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            full_playbook_path = f"{playbook_path}/{playbook}"

            for ifs in sliver.interface_info.interfaces.values():
                self.__attach_detach_port(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                          vlan=sliver.label_allocations.vlan, interface_sliver=ifs,
                                          attach=True, raise_exception=True)
        except Exception as e:
            # Delete Ports in case of failure
            if sliver is not None and unit_id is not None:
                self.__cleanup(sliver=sliver)
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0,
                      Constants.PROPERTY_EXCEPTION_MESSAGE: e}
            self.get_logger().error(e)
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info(f"Create completed")
        return result, unit

    def modify(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Handle Modify for Network Services for OpenStack vNIC
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Modify invoked for unit: {unit}")
            current_sliver = unit.get_sliver()
            modified_sliver = unit.get_modified()

            if not isinstance(current_sliver, NetworkServiceSliver) or not isinstance(modified_sliver, NetworkServiceSliver):
                raise VnicNetHandlerException(f"Invalid Sliver type {type(current_sliver)}  {type(modified_sliver)}")

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][str(current_sliver.get_type())]

            if playbook is None or inventory_path is None:
                raise VnicNetHandlerException(f"Missing config parameters playbook: {playbook} "
                                              f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            full_playbook_path = f"{playbook_path}/{playbook}"

            diff = current_sliver.diff(other_sliver=modified_sliver)
            if diff is None:
                self.get_logger().info("Nothing to do!")
                return result, unit

            # Modify topology
            for x in diff.added.interfaces:
                interface = modified_sliver.interface_info.interfaces[x]
                self.__attach_detach_port(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                          interface_sliver=interface,  vlan=current_sliver.label_allocations.vlan,
                                          attach=True)

            for x in diff.removed.interfaces:
                interface = current_sliver.interface_info.interfaces[x]
                self.__attach_detach_port(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                          interface_sliver=interface, vlan=current_sliver.label_allocations.vlan,
                                          attach=False)
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

    def delete(self, unit: ConfigToken) -> Tuple[dict, ConfigToken]:
        """
        Handle delete for Network Services for OpenStack vNIC
        """
        result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                  Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_OK,
                  Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        try:
            self.get_logger().info(f"Delete invoked for unit: {unit}")
            sliver = unit.get_sliver()
            if sliver is None:
                raise VnicNetHandlerException(f"Unit # {unit} has no assigned slivers")

            if not isinstance(sliver, NetworkServiceSliver):
                raise VnicNetHandlerException(f"Invalid Sliver type {type(sliver)}")

            self.__cleanup(sliver=sliver)
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

    def __cleanup(self, *, sliver: NetworkServiceSliver, raise_exception: bool = False):
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][str(sliver.get_type())]

            if playbook is None or inventory_path is None:
                raise VnicNetHandlerException(f"Missing config parameters playbook: {playbook} "
                                              f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            full_playbook_path = f"{playbook_path}/{playbook}"

            for ifs in sliver.interface_info.interfaces.values():
                self.__attach_detach_port(playbook_path=full_playbook_path, inventory_path=inventory_path,
                                          interface_sliver=ifs, attach=False, vlan=sliver.label_allocations.vlan,
                                          raise_exception=raise_exception)
        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup {sliver.get_name()} error: {e}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e

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

    def __attach_detach_port(self, *, playbook_path: str, inventory_path: str, interface_sliver: InterfaceSliver,
                             vlan: str, attach: bool = True, raise_exception: bool = False):
        """
        Invoke ansible playbook to attach/detach a vNIC device to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param interface_sliver: Interface Sliver
        :param vlan: VLAN
        :param attach: True for attach and False for detach
        :return:
        """
        self.get_logger().debug("__attach_detach_port IN")
        try:
            network_name_prefix = self.get_config()[AmConstants.RUNTIME_SECTION][AmConstants.NETWORK_NAME_PREFIX]

            extra_vars = {AmConstants.VM_NAME: interface_sliver.labels.instance_parent,
                          AmConstants.PORT_NAME: f"{interface_sliver.labels.instance_parent}-{interface_sliver.get_name()}",
                          AmConstants.NETWORK_NAME: f"{network_name_prefix}-{vlan}",
                          AmConstants.MAC: interface_sliver.labels.mac}
            if interface_sliver.flags is not None and interface_sliver.flags.auto_config:
                if interface_sliver.label_allocations.ipv4 is not None:
                    extra_vars[AmConstants.IP_ADDRESS] = interface_sliver.label_allocations.ipv4
                elif interface_sliver.label_allocations.ipv6 is not None:
                    extra_vars[AmConstants.IP_ADDRESS] = interface_sliver.label_allocations.ipv6
            if attach:
                extra_vars[AmConstants.PORT_PROV_OP] = AmConstants.PROV_OP_CREATE
            else:
                extra_vars[AmConstants.PORT_PROV_OP] = AmConstants.PROV_OP_DELETE

            self.get_logger().info(f"Provisioning Interface: {interface_sliver}")
            self.__execute_ansible(inventory_path=inventory_path, playbook_path=playbook_path,
                                   extra_vars=extra_vars)
        except Exception as e:
            self.get_logger().error(f"Error occurred attach:{attach}/detach: {not attach} interface: {interface_sliver}")
            self.get_logger().error(traceback.format_exc())
            if raise_exception:
                raise e
        finally:
            self.get_logger().debug("__attach_detach_port OUT")
