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
import logging
import multiprocessing

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.core.unit import Unit
from fabric_cf.actor.core.util.id import ID
from fim.slivers import InstanceCatalog
from fim.slivers.attached_components import ComponentSliver, ComponentType, AttachedComponentsInfo
from fim.slivers.capacities_labels import Capacities, Labels, CapacityHints
from fim.slivers.network_node import NodeSliver, NodeType

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants


class TestPlaybooks:
    logger = logging.getLogger(__name__)
    log_format = \
        '%(asctime)s - %(name)s - {%(filename)s:%(lineno)d} - [%(threadName)s] - %(levelname)s - %(message)s'
    logging.basicConfig(handlers=[logging.StreamHandler()], format=log_format, force=True)
    logger.setLevel("DEBUG")

    prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
    lock = multiprocessing.Lock()
    handler = VMHandler(logger=logger, properties=prop, process_lock=lock)
    from fabric_cf.actor.core.container.globals import GlobalsSingleton
    GlobalsSingleton.get().log = logger

    @staticmethod
    def create_unit(include_pci: bool = True, include_image: bool = True, include_name: bool = True,
                    include_instance_name: bool = False) -> Unit:
        """
        Create a unit
        :param include_pci:
        :param include_image:
        :param include_name:
        :param include_instance_name:
        :return:
        """
        u = Unit(rid=ID(uid='0a0c2fb9-071a-4a3a-ba94-aa178c237aa2'))
        sliver = NodeSliver()
        cap = Capacities(core=2, ram=8, disk=10)
        sliver.set_properties(type=NodeType.VM, site="UKY", capacity_allocations=cap)
        sliver.label_allocations = Labels(instance_parent="uky-w1.fabric-testbed.net")
        catalog = InstanceCatalog()
        instance_type = catalog.map_capacities_to_instance(cap=cap)
        cap_hints = CapacityHints(instance_type=instance_type)
        sliver.set_properties(capacity_hints=cap_hints,
                              capacity_allocations=catalog.get_instance_capacities(instance_type=instance_type))

        if include_name:
            sliver.set_properties(name="uky-w1-b")

        if include_image:
            sliver.set_properties(image_type='qcow2', image_ref='default_fedora')

        if include_pci:
            component = ComponentSliver()
            labels = Labels(bdf=["0000:41:00.0", "0000:41:00.1"])
            component.set_properties(type=ComponentType.SmartNIC, model='ConnectX-5', name='nic1',
                                     label_allocations=labels)
            sliver.attached_components_info = AttachedComponentsInfo()
            sliver.attached_components_info.add_device(device_info=component)

        if include_instance_name:
            sliver.label_allocations.instance = "instance-00001077"

        u.set_sliver(sliver=sliver)
        return u

    def test_create_vm_success(self):
        """
        Test successful creation of VM with PCI devices
        :return:
        """
        u = self.create_unit()

        r, u = self.handler.create(unit=u)
        print(r)
        print(u.sliver)
        return u

    def test_create_vm_success_no_pci(self):
        """
        Test successful creation of VM without PCI devices
        :return:
        """
        u = self.create_unit(include_pci=False)

        r, u = self.handler.create(unit=u)
        print(r)
        print(u.sliver)

    def test_delete_vm_success(self, u):
        """
        Test successful deletion of a VM
        :return:
        """
        if u is None:
            u = self.create_unit(include_instance_name=True, include_name=True)

        r, u = self.handler.delete(unit=u)
        print(r)
        print(u.get_sliver())

    def test_delete_vm_success_no_pci(self):
        """
        Test successful deletion of a VM without PCI devices
        :return:
        """
        u = self.create_unit(include_pci=False)

        r, u = self.handler.delete(unit=u)
        print(r)
        print(u.get_sliver())

    def test_config_nw_interface(self):
        self.handler.configure_network_interface(mgmt_ip="128.163.179.50", user=AmConstants.CENTOS,
                                                 resource_type=str(ComponentType.SmartNIC),
                                                 mac_address="0C:42:A1:78:F8:04",
                                                 ipv4_address="192.168.11.2")

    def test_config_nw_interface_tagged(self):
        self.handler.configure_network_interface(mgmt_ip="128.163.179.50", user=AmConstants.CENTOS,
                                                 resource_type=str(ComponentType.SmartNIC),
                                                 mac_address="0C:42:A1:78:F8:04",
                                                 ipv4_address="192.168.11.2", vlan="200")

    def test_poa_cpuinfo(self):
        u = self.create_unit(include_instance_name=True, include_name=True)
        data = {"operation": "cpuinfo"}
        self.handler.poa(unit=u, data=data)

    def test_poa_numainfo(self):
        u = self.create_unit(include_instance_name=True, include_name=True)
        data = {"operation": "numainfo"}
        self.handler.poa(unit=u, data=data)

    def test_poa_numatune(self):
        u = self.create_unit(include_instance_name=True, include_name=True)
        data = {"node_set": ["1", "2", "3"], "operation": "numatune"}
        self.handler.poa(unit=u,data=data)

    def test_poa_reboot(self):
        u = self.create_unit(include_instance_name=True, include_name=True)
        data = {"operation": "reboot"}
        self.handler.poa(unit=u, data=data)

    def test_poa_cpupin(self):
        u = self.create_unit(include_instance_name=True, include_name=True)
        data = {"vcpu_cpu_map": [{"vcpu": 0, "cpu": 34}, {"vcpu": 1, "cpu": 35}], "operation": "cpupin"}
        self.handler.poa(unit=u, data=data)

    def test_fpga_prov(self):
        u = Unit(rid=ID(uid='0a0c2fb9-071a-4a3a-ba94-aa178c237aa2'))
        u.properties = {Constants.USER_SSH_KEY:
                        "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQDIxGUVBf24l4gSgUtQQaScP7S604CpXKh66cCMZB1GoXfGqyhRVO1xQUXGA2Oj8MeZf3bo4tjmrPnVeeTVfwTrxkkFNvekwY4QbGX7o8YPNnEFquLWMmkoLn9RFJI47Cj+JHWQN7sEW4WVnmHNITcw5lD3V+yw1bD5M0boUXvh/MnHTu59MEDRyLUyWY+N1FUxHrO0UgSISczRjFS31zF5WY83ssNWq+zxD0NM6GhLWg5Ynzat1J75NRvnMVkuj0VmFcJuHIl3jYCdL9uE7kCw08oh06p/VZBzUIDP6EB0e+H1udu0DvT7SunqBZnobrTCyj1Bma9BJEHPhocIIcPl komalthareja@dhcp152-54-6-178.wireless.europa.renci.org,"
                        "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQDIxGUVBf24l4gSgUtQQaScP7S604CpXKh66cCMZB1GoXfGqyhRVO1xQUXGA2Oj8MeZf3bo4tjmrPnVeeTVfwTrxkkFNvekwY4QbGX7o8YPNnEFquLWMmkoLn9RFJI47Cj+JHWQN7sEW4WVnmHNITcw5lD3V+yw1bD5M0boUXvh/MnHTu59MEDRyLUyWY+N1FUxHrO0UgSISczRjFS31zF5WY83ssNWq+zxD0NM6GhLWg5Ynzat1J75NRvnMVkuj0VmFcJuHIl3jYCdL9uE7kCw08oh06p/VZBzUIDP6EB0e+H1udu0DvT7SunqBZnobrTCyj1Bma9BJEHPhocIIcPl komalthareja@dhcp152-54-6-178.wireless.europa.renci.org"}
        sliver = NodeSliver()
        cap = Capacities(core=2, ram=8, disk=10)
        sliver.set_properties(type=NodeType.VM, site="RENC", capacity_allocations=cap, name="fpga-vm")
        sliver.label_allocations = Labels(instance_parent="renc-w2.fabric-testbed.net")
        catalog = InstanceCatalog()
        instance_type = catalog.map_capacities_to_instance(cap=cap)
        cap_hints = CapacityHints(instance_type=instance_type)
        sliver.set_properties(capacity_hints=cap_hints,
                              capacity_allocations=catalog.get_instance_capacities(instance_type=instance_type))
        sliver.set_properties(image_type='qcow2', image_ref='docker_rocky_8')

        component = ComponentSliver()
        labels = Labels(bdf=["0000:25:00.0", "0000:25:00.1"])
        component.set_properties(type=ComponentType.FPGA, model='Xilinx', name='nic1',
                                 label_allocations=labels, labels=labels)
        sliver.attached_components_info = AttachedComponentsInfo()
        sliver.attached_components_info.add_device(device_info=component)
        u.set_sliver(sliver=sliver)
        r, u = self.handler.create(unit=u)


if __name__ == "__main__":
    import time
    tpb = TestPlaybooks()
    #tpb.test_create_vm_success_no_pci()

    #time.sleep(10)
    #tpb.test_delete_vm_success_no_pci()
    #time.sleep(10)
    #u = tpb.test_create_vm_success()

    #time.sleep(10)
    #tpb.test_delete_vm_success(u=u)
    #tpb.test_config_nw_interface_tagged()
    #tpb.test_config_nw_interface()

    #tpb.test_poa_cpuinfo()
    #tpb.test_poa_numainfo()

    #tpb.test_poa_numatune()
    #tpb.test_poa_cpupin()
    #tpb.test_poa_reboot()

    tpb.test_fpga_prov()
