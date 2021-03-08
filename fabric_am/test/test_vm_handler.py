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
from fim.slivers.capacities_labels import Capacities, Labels
from fim.slivers.network_node import NodeSliver, NodeType

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants


class TestVmHandler(unittest.TestCase):
    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s",
                        handlers=[logging.StreamHandler()])

    def create_unit(self, include_pci: bool = True, include_image: bool = True, include_name: bool = True,
                    include_instance_name: bool = False) -> Unit:
        u = Unit(uid=ID(uid='u1'))
        sliver = NodeSliver()
        cap = Capacities()
        cap.set_fields(core=4, ram=64, disk=500)
        sliver.set_properties(resource_type=NodeType.VM, site="RENC", capacities=cap)
        sliver.worker_node_name = "renc-w1.fabric-testbed.net"

        if include_name:
            sliver.set_properties(name="n1")

        if include_image:
            sliver.set_properties(image_type='qcow2',image_ref='default_centos_8')

        if include_pci:
            component = ComponentSliver()
            labels = Labels()
            labels.set_fields(bdf='0,0,85')
            component.set_properties(resource_type=ComponentType.SmartNIC, model='ConnectX-6', name='nic1', labels=labels)
            sliver.attached_components_info = AttachedComponentsInfo()
            sliver.attached_components_info.add_device(device_info=component)

        if include_instance_name:
            sliver.instance_name = "instance-001"

        u.set_sliver(sliver=sliver)
        return u

    def test_create_vm_success(self):
        u = self.create_unit()

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
        self.assertIsNotNone(u.sliver.instance_name)
        self.assertIsNotNone(u.sliver.management_ip)
        self.assertIsNotNone(u.sliver.management_interface_mac_address)
        self.assertEqual(u.sliver.state, "active")

    def test_create_vm_success_no_pci(self):
        u = self.create_unit(include_pci=False)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
        self.assertIsNotNone(u.sliver.instance_name)
        self.assertIsNotNone(u.sliver.management_ip)
        self.assertIsNotNone(u.sliver.management_interface_mac_address)
        self.assertEqual(u.sliver.state, "active")

    def test_create_vm_fail_no_image(self):
        u = self.create_unit(include_image=False)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)
        self.assertIsNone(u.sliver.instance_name)
        self.assertIsNone(u.sliver.management_ip)
        self.assertIsNone(u.sliver.management_interface_mac_address)
        self.assertIsNone(u.sliver.state)

    def test_create_vm_fail_no_config(self):
        u = self.create_unit()

        prop = {}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.create(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)
        self.assertIsNone(u.sliver.instance_name)
        self.assertIsNone(u.sliver.management_ip)
        self.assertIsNone(u.sliver.management_interface_mac_address)
        self.assertIsNone(u.sliver.state)

    def test_delete_vm_success(self):
        u = self.create_unit()

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_delete_vm_success_no_pci(self):
        u = self.create_unit(include_pci=False)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_delete_vm_fail_no_config(self):
        u = self.create_unit()

        prop = {}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)

    def test_delete_vm_fail_no_vm_name(self):
        u = self.create_unit(include_name=False)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.delete(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)

    def test_modify_fail_no_instance_name(self):
        u = self.create_unit()

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.modify(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_MODIFY)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)

    def test_modify_vm_fail_no_config(self):
        u = self.create_unit()

        prop = {}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.modify(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_MODIFY)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)
        self.assertIsNotNone(r[Constants.PROPERTY_EXCEPTION_MESSAGE])

    def test_modify_vm_success(self):
        u = self.create_unit(include_instance_name=True)

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
        handler = VMHandler(logger=self.logger, properties=prop)

        r, u = handler.modify(unit=u)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_MODIFY)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
