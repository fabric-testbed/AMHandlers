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

from fabric_cf.actor.core.core.unit import Unit
from fabric_cf.actor.core.util.id import ID
from fim.slivers import InstanceCatalog
from fim.slivers.attached_components import ComponentSliver, ComponentType, AttachedComponentsInfo
from fim.slivers.capacities_labels import Capacities, Labels, CapacityHints
from fim.slivers.network_node import NodeSliver, NodeType

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants


class TestPlaybooks:
    log_config = {"log-directory": ".", "log-file": "handler.log","log-level": "DEBUG","log-retain": 5,
                  "log-size": 5000000, "logger": __name__}

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
        u = Unit(rid=ID(uid='rid-1'))
        sliver = NodeSliver()
        cap = Capacities()
        cap.set_fields(core=2, ram=8, disk=10)
        sliver.set_properties(type=NodeType.VM, site="RENC", capacity_allocations=cap)
        sliver.label_allocations = Labels().set_fields(instance_parent="renc-w3")
        catalog = InstanceCatalog()
        instance_type = catalog.map_capacities_to_instance(cap=cap)
        cap_hints = CapacityHints().set_fields(instance_type=instance_type)
        sliver.set_properties(capacity_hints=cap_hints,
                              capacity_allocations=catalog.get_instance_capacities(instance_type=instance_type))

        if include_name:
            sliver.set_properties(name="n2")

        if include_image:
            sliver.set_properties(image_type='qcow2', image_ref='default_fedora')

        if include_pci:
            component = ComponentSliver()
            labels = Labels()
            labels.set_fields(bdf=["0000:41:00.0", "0000:41:00.1"])
            component.set_properties(type=ComponentType.SmartNIC, model='ConnectX-5', name='nic1',
                                     label_allocations=labels)
            #labels.set_fields(bdf="0000:81:00.0")
            #component.set_properties(type=ComponentType.GPU, model='Tesla T4', name='nic12', label_allocations=labels)
            sliver.attached_components_info = AttachedComponentsInfo()
            sliver.attached_components_info.add_device(device_info=component)

        if include_instance_name:
            sliver.label_allocations.set_fields(instance="instance-001")

        u.set_sliver(sliver=sliver)
        return u

    def test_create_vm_success(self):
        """
        Test successful creation of VM with PCI devices
        :return:
        """
        u = self.create_unit()

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)

        r, u = handler.create(unit=u)
        print(r)
        print(u.sliver)
        return u

    def test_create_vm_success_no_pci(self):
        """
        Test successful creation of VM without PCI devices
        :return:
        """
        u = self.create_unit(include_pci=False)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)

        r, u = handler.create(unit=u)
        print(r)
        print(u.sliver)

    def test_delete_vm_success(self, u):
        """
        Test successful deletion of a VM
        :return:
        """
        if u is None:
            u = self.create_unit(include_instance_name=True, include_name=True)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)

        r, u = handler.delete(unit=u)
        print(r)
        print(u.get_sliver())

    def test_delete_vm_success_no_pci(self):
        """
        Test successful deletion of a VM without PCI devices
        :return:
        """
        u = self.create_unit(include_pci=False)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)

        r, u = handler.delete(unit=u)
        print(r)
        print(u.get_sliver())

    def test_config_nw_interface(self):
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)
        handler.configure_network_interface(mgmt_ip="152.54.15.35", user=AmConstants.CENTOS_DEFAULT_USER,
                                            resource_type=str(ComponentType.SharedNIC), mac_address="ba:93:fc:bb:37:e0",
                                            ipv4_address="192.168.10.51/24")

    def test_config_nw_interface_tagged(self):
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)
        handler.configure_network_interface(mgmt_ip="152.54.15.35", user=AmConstants.CENTOS_DEFAULT_USER,
                                            resource_type=str(ComponentType.SharedNIC), mac_address="ba:93:fc:bb:37:e0",
                                            ipv4_address="192.168.10.51/24", vlan="200")

    def attach_fip(self):
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(log_config=self.log_config, properties=prop)
        res = handler.__attach_fip(playbook_path='fabric_am/playbooks/head_vm_provisioning.yml',
                                   inventory_path='fabric_am/playbooks/inventory',
                                   vm_name="test", unit_id="komal")
        print(res)


if __name__ == "__main__":
    import time
    tpb = TestPlaybooks()
    #tpb.test_create_vm_success_no_pci()

    #time.sleep(10)
    #tpb.test_delete_vm_success_no_pci()
    #time.sleep(10)
    u = tpb.test_create_vm_success()

    #time.sleep(10)
    #tpb.test_delete_vm_success(u=u)
    #tpb.test_config_nw_interface_tagged()
    #tpb.test_config_nw_interface()

