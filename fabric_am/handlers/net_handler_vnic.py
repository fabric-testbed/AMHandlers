import traceback
from typing import Tuple

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase
from fim.slivers.interface_info import InterfaceSliver
from fim.slivers.network_service import NetworkServiceSliver, ServiceType

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class NetHandlerVnicException(Exception):
    """
    vNIC Net Handler Exception
    """
    pass


class NetHandlerVnic(HandlerBase):
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
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            cleanup_section = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_CLEANUP]
            cleanup_playbook = f"{playbook_path}/{cleanup_section[AmConstants.CLEAN_ALL]}"
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]
            extra_vars = {AmConstants.PORT_PROV_OP: AmConstants.PROV_OP_DELETE_ALL}

            self.__execute_ansible(inventory_path=inventory_path, playbook_path=cleanup_playbook,
                                   extra_vars=extra_vars)
        except Exception as e:
            self.get_logger().error(f"Failure to clean up existing ports: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
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
                raise NetHandlerVnicException(f"Unit # {unit} has no assigned slivers")

            if not isinstance(sliver, NetworkServiceSliver):
                raise NetHandlerVnicException(f"Invalid Sliver type {type(sliver)}")

            if sliver.get_type() != ServiceType.L2Bridge:
                raise NetHandlerVnicException(f"Unsupported Service - {sliver.get_type()}")

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            for ifs in sliver.interface_info.interfaces.values():
                self.__attach_detach_port(playbook_path=playbook_path, inventory_path=inventory_path,
                                          interface_sliver=ifs, unit_id=unit_id, attach=True)

        except Exception as e:
            # Delete Ports in case of failure
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
                raise NetHandlerVnicException(f"Invalid Sliver type {type(current_sliver)}  {type(modified_sliver)}")

            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            diff = current_sliver.diff(other_sliver=modified_sliver)
            if diff is None:
                self.get_logger().info("Nothing to do!")
                return result, unit

            # Modify topology
            for x in diff.added.interfaces:
                interface = modified_sliver.interface_info.interfaces[x]
                self.__attach_detach_port(playbook_path=playbook_path, inventory_path=inventory_path,
                                          interface_sliver=interface, unit_id=str(unit.get_reservation_id()),
                                          attach=True)

            for x in diff.removed.interfaces:
                interface = modified_sliver.interface_info.interfaces[x]
                self.__attach_detach_port(playbook_path=playbook_path, inventory_path=inventory_path,
                                          interface_sliver=interface, unit_id=str(unit.get_reservation_id()),
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
                raise NetHandlerVnicException(f"Unit # {unit} has no assigned slivers")

            if not isinstance(sliver, NetworkServiceSliver):
                raise NetHandlerVnicException(f"Invalid Sliver type {type(sliver)}")

            self.__cleanup(sliver=sliver, unit_id=str(unit.get_reservation_id()))
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

    def __cleanup(self, *, sliver: NetworkServiceSliver, unit_id: str, raise_exception: bool = False):
        try:
            playbook_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_LOCATION]
            inventory_path = self.get_config()[AmConstants.PLAYBOOK_SECTION][AmConstants.PB_INVENTORY]

            for ifs in sliver.interface_info.interfaces.values:
                self.__attach_detach_port(playbook_path=playbook_path, inventory_path=inventory_path,
                                          interface_sliver=ifs, unit_id=unit_id, attach=False,
                                          raise_exception=raise_exception)
        except Exception as e:
            self.get_logger().error(f"Exception occurred in cleanup {unit_id} error: {e}")
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
                             unit_id: str, attach: bool = True, raise_exception: bool = False):
        """
        Invoke ansible playbook to attach/detach a vNIC device to a provisioned VM
        :param playbook_path: playbook location
        :param inventory_path: inventory location
        :param interface_sliver: Interface Sliver
        :param unit_id: Reservation Id
        :param attach: True for attach and False for detach
        :return:
        """
        self.get_logger().debug("__attach_detach_port IN")
        try:
            resource_type = f"{interface_sliver.get_type()}-{interface_sliver.get_model()}"
            playbook = self.get_config()[AmConstants.PLAYBOOK_SECTION][resource_type]
            if playbook is None or inventory_path is None:
                raise NetHandlerVnicException(f"Missing config parameters playbook: {playbook} "
                                              f"playbook_path: {playbook_path} inventory_path: {inventory_path}")

            full_playbook_path = f"{playbook_path}/{playbook}"

            extra_vars = {AmConstants.VM_NAME: interface_sliver.labels.instance_parent,
                          AmConstants.PORT_NAME: f"{unit_id}-{interface_sliver.get_name()}"}
            if attach:
                extra_vars[AmConstants.PORT_PROV_OP] = AmConstants.PROV_OP_CREATE
            else:
                extra_vars[AmConstants.PORT_PROV_OP] = AmConstants.PROV_OP_DELETE

            self.get_logger().info(f"Provisioning Interface: {interface_sliver}")
            self.__execute_ansible(inventory_path=inventory_path, playbook_path=full_playbook_path,
                                   extra_vars=extra_vars)
        except Exception as e:
            self.get_logger().error(f"Error occurred attach:{attach}/detach: {not attach} interface: {interface_sliver}")
            if raise_exception:
                raise e
        finally:
            self.get_logger().debug("__attach_detach_port OUT")