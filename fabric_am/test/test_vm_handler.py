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
import unittest

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.core.unit import Unit
from fabric_cf.actor.core.util.id import ID
from fim.slivers.attached_components import ComponentSliver, ComponentType, AttachedComponentsInfo
from fim.slivers.capacities_labels import Capacities, Labels, CapacityHints
from fim.slivers.instance_catalog import InstanceCatalog
from fim.slivers.interface_info import InterfaceInfo, InterfaceSliver
from fim.slivers.network_node import NodeSliver, NodeType
from fim.slivers.network_service import NetworkServiceInfo, NetworkServiceSliver
from fim.user.network_service import NetworkService

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants


class TestVmHandler(unittest.TestCase):
    logger = logging.getLogger(__name__)
    log_format = \
        '%(asctime)s - %(name)s - {%(filename)s:%(lineno)d} - [%(threadName)s] - %(levelname)s - %(message)s'
    logging.basicConfig(handlers=[logging.StreamHandler()], format=log_format, force=True)

    prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
    handler = VMHandler(logger=logger, properties=prop)
    handler.test_mode = True


    @staticmethod
    def create_unit(include_pci: bool = True, include_image: bool = True, include_name: bool = True,
                    include_instance_name: bool = False, include_ns: bool = False) -> Unit:
        """
        Create a unit
        :param include_pci:
        :param include_image:
        :param include_name:
        :param include_instance_name:
        :param include_ns:
        :return:
        """
        u = Unit(rid=ID(uid="rid-1"))
        sliver = NodeSliver()
        cap = Capacities()
        cap.set_fields(core=4, ram=64, disk=500)
        catalog = InstanceCatalog()
        instance_type = catalog.map_capacities_to_instance(cap=cap)
        cap_hints = CapacityHints().set_fields(instance_type=instance_type)
        sliver.set_properties(type=NodeType.VM, site="RENC",
                              capacity_hints=cap_hints,
                              capacity_allocations=catalog.get_instance_capacities(instance_type=instance_type))
        sliver.label_allocations = Labels().set_fields(instance_parent="renc-w1.fabric-testbed.net")

        if include_name:
            sliver.set_properties(name="n1")

        if include_image:
            sliver.set_properties(image_type='qcow2', image_ref='default_centos_8')

        if include_pci:
            component = ComponentSliver()
            labels = Labels()
            labels.set_fields(bdf=["0000:41:00.0", "0000:41:00.1"])
            component.set_properties(type=ComponentType.SmartNIC, model='ConnectX-6', name='nic1',
                                     label_allocations=labels)
            sliver.attached_components_info = AttachedComponentsInfo()
            sliver.attached_components_info.add_device(device_info=component)

        if include_instance_name:
            sliver.label_allocations.set_fields(instance="instance-001")

        if include_ns:
            sliver.network_service_info = NetworkServiceInfo()
            ns = NetworkServiceSliver()
            ns.interface_info = InterfaceInfo()
            ifs1 = InterfaceSliver()
            c = Capacities()
            c.bw = 100
            c.unit = 1
            l1 = Labels()
            l1.ipv4 = '192.168.11.3'
            l1.vlan = '200'
            l1.local_name = 'p1'
            la_1 = Labels()
            la_1.mac = '0C:42:A1:EA:C7:51'
            la_1.vlan = '200'
            ifs1.capacities = c
            ifs1.labels = l1
            ifs1.label_allocations = la_1


            ifs2 = InterfaceSliver()
            ifs2.capacities = c
            l2 = Labels()
            l2.ipv4 = '192.168.11.2'
            l2.local_name = 'p2'
            la_2 = Labels()
            la_2.mac = '0C:42:A1:EA:C7:52'

            ifs2.labels = l2
            ifs2.label_allocations = la_1

            ns.interface_info.interfaces = {'ifs1': ifs1, 'ifs2': ifs2}
            sliver.network_service_info.network_services = {'ns1': ns}

        u.set_sliver(sliver=sliver)
        return u

    def test_create_vm_success(self):
        """
        Test successful creation of VM with PCI devices
        :return:
        """
        u = self.create_unit()

        r, u = self.handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
        self.assertIsNotNone(u.sliver.label_allocations.instance)
        self.assertIsNotNone(u.sliver.management_ip)

    def test_create_vm_success_no_pci(self):
        """
        Test successful creation of VM without PCI devices
        :return:
        """
        u = self.create_unit(include_pci=False)

        r, u = self.handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
        self.assertIsNotNone(u.sliver.label_allocations.instance)
        self.assertIsNotNone(u.sliver.management_ip)

    def test_create_vm_fail_no_image(self):
        """
        Test failure to create VM when no image is specified
        :return:
        """
        u = self.create_unit(include_image=False)

        r, u = self.handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)
        self.assertIsNone(u.sliver.label_allocations.instance)
        self.assertIsNone(u.sliver.management_ip)

    def test_create_vm_fail_no_config(self):
        """
        Test failure to create VM when no handler config is specified
        :return:
        """
        u = self.create_unit()

        prop = {}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)
        self.assertIsNone(u.sliver.label_allocations.instance)
        self.assertIsNone(u.sliver.management_ip)

    def test_delete_vm_success(self):
        """
        Test successful deletion of a VM
        :return:
        """
        u = self.create_unit(include_instance_name=True, include_name=True)

        r, u = self.handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_delete_vm_success_no_pci(self):
        """
        Test successful deletion of a VM without PCI devices
        :return:
        """
        u = self.create_unit(include_pci=False)

        r, u = self.handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_delete_vm_fail_no_config(self):
        """
        Test failure to delete VM when no handler config is specified
        :return:
        """
        u = self.create_unit()

        prop = {}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)

    def test_delete_vm_fail_no_vm_name(self):
        """
        Test failure to delete VM when no VM Name is specified
        :return:
        """
        u = self.create_unit(include_name=False)

        r, u = self.handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)

    def test_modify_vm_success(self):
        """
        Test successfully modify VM
        :return:
        """
        u = self.create_unit(include_instance_name=True)

        r, u = self.handler.modify(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_MODIFY)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_create_vm_with_nic_config(self):
        u = self.create_unit(include_ns=True)

        r, u = self.handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
        self.assertIsNotNone(u.sliver.label_allocations.instance)
        self.assertIsNotNone(u.sliver.management_ip)
