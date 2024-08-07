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
# Author: Ilya Baldin (ibaldin@renci.org), Xi Yang (xiwang@es.net), Komal Thareja (kthare10@renci.org)
import logging
import threading
import unittest
import time
import uuid

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.core.unit import Unit
from fabric_cf.actor.core.util.id import ID
from fim.slivers.attached_components import ComponentSliver, ComponentType, AttachedComponentsInfo
from fim.slivers.capacities_labels import Capacities, Labels, CapacityHints
from fim.slivers.instance_catalog import InstanceCatalog
from fim.slivers.network_service import NetworkServiceSliver, ServiceType, NSLayer, MirrorDirection
from fim.slivers.interface_info import InterfaceSliver, InterfaceType, InterfaceInfo
from fim.slivers.path_info import Path, PathInfo, ERO
from fim.slivers.gateway import Gateway

# FIXME: @xiyang
from fabric_am.handlers.net_handler import NetHandler
from fabric_am.util.am_constants import AmConstants


class TestNetHandler(unittest.TestCase):
    logger = logging.getLogger(__name__)
    log_format = \
        '%(asctime)s - %(name)s - {%(filename)s:%(lineno)d} - [%(threadName)s] - %(levelname)s - %(message)s'
    logging.basicConfig(handlers=[logging.StreamHandler()], format=log_format, force=True)

    def setUp(self) -> None:
        self.unit = Unit(rid=ID(uid="rid-1"))

    def test_L2Bridge(self):
        # create a NetworkService sliver for L2Bridge
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}
        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for L2Bridge and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L2BridgeServiceTest')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.L2Bridge)
        sliver.set_layer(NSLayer.L2)

        # Interface properties
        #
        # The service definitions make a distinction between interface which requires
        # type = parse(InterfaceSliver.Labels.local_name)
        # id = parse(InterfaceSliver.Labels.local_name)
        # outervlan = InterfaceSliver.Labels.vlan
        # innervlan = InterfaceSliver.Labels.inner_vlan
        # bw = InterfaceSliver.Capacities.bw (0 - best-effort)
        # burst size = InterfaceSliver.Capacities.burst_size
        #
        # and STP which in addition also requires NSO device name.
        # In deep network sliver NSO Device name goes on *each* interface, then handler.create can parse
        # out the interfaces and figure out which STP each interface goes with based on that.
        # NSO device name = InterfaceSliver.Labels.device_name
        #
        # The properties of InterfaceSlivers noted above must be copied by Orchestrator from various places
        # a) the switch TrunkPort port the ASM ServicePort maps to in CBM
        # b) the Shared or Dedicated ASM port on the card the ServicePort peers with in ASM
        # c) the Shared or Dedicated CBM port the peer ASM port maps to
        # Below for each property comments indicate where they come from by a, b, c

        # Orchestrator determines peer ports in ASM (between ServicePort and corresponding Shared/Dedicated card port)
        # and sets nodemaps to point from ASM ServicePort to corresponding CBM TrunkPort
        # as well as between Shared/Dedicated ASM port on the NIC and the corresponding CBM Shared/Dedicated port

        #
        # create a small number of Interface slivers, set their properties and link to service
        #
        isl1 = InterfaceSliver()
        # the name is set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(vlan='100', local_name='HundredGigE0/0/0/17', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)
        # inner_vlan - not used for now - user would fill it in directly on the sliver Labels -

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (comments for field info origin omitted below)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(local_name='TwentyFiveGigE0/0/0/23/1', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        # sliver_labels = Labels.update(sliver_labels, vlan='102')

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi
        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L2Bridge')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_L2Bridge_Hairpin(self):
        # create a NetworkService sliver for L2Bridge
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for L2Bridge and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L2-UKY-Hairpin')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.L2Bridge)
        sliver.set_layer(NSLayer.L2)

        #
        # create a small number of Interface slivers, set their properties and link to service
        #
        isl1 = InterfaceSliver()
        # the name is set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        # inner_vlan - not used for now - user would fill it in directly on the sliver Labels -
        # need to discuss.
        # sl1labs.set_fields(inner_vlan='3')
        # vlan - source: (c)
        # local_name source: (a)
        # NSO device name source: (a) - need to find the owner switch of the network service in CBM
        # and take its .name or labels.local_name
        sliver_labels = Labels(vlan='101', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (comments for field info origin omitted below)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(vlan='102', inner_vlan='200', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi
        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L2Bridge')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_L2Bridge_Modify(self):
        # create a NetworkService sliver for L2Bridge
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for L2Bridge and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L2BridgeServiceTest')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.L2Bridge)
        sliver.set_layer(NSLayer.L2)

        isl1 = InterfaceSliver()
        # the name is set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(vlan='2091', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (comments for field info origin omitted below)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(vlan='2092', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi
        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L2Bridge')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        modified_sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        modified_sliver.set_name('L2BridgeServiceTest')  # must be the same name
        modified_sliver.set_type(ServiceType.L2Bridge)
        modified_sliver.set_layer(NSLayer.L2)

        ifi = InterfaceInfo()  # started over
        ifi.add_interface(isl1)  # Add Interface1 (reused)

        # add Interface3
        isl3 = InterfaceSliver()
        isl3.set_name('Interface3')
        isl3.set_type(InterfaceType.ServicePort)
        sliver_labels = Labels(vlan='200', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)
        isl3.set_labels(sliver_labels)
        isl3.set_capacities(sliver_capacities)

        ifi.add_interface(isl3)
        modified_sliver.interface_info = ifi

        updated_unit.set_modified(modified=modified_sliver)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.modify(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_MODIFY)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_L2PTP(self):
        # create a NetworkService sliver for L2PTP
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())

        #
        # create a network sliver for L2Bridge and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name - can we use the sliver name - only guaranteed unique in the slice
        sliver.set_name('L2PTPServiceTest')
        sliver.set_type(ServiceType.L2PTP)
        sliver.set_layer(NSLayer.L2)

        # ERO
        # first declare a path. Each path is a list of somethings. a2z and z2a maintained separately within Path
        ero_path = Path()

        ero_path.set_symmetric(["10.1.1.1", "10.1.1.2"])
        # default is loose ERO, set strict=True if want otherwise
        ero = ERO(strict=False)
        ero.set(ero_path)
        sliver.set_ero(ero)

        #
        # STP_A interface
        #
        stp_a = InterfaceSliver()
        stp_a.set_name('Interface1')
        stp_a.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(vlan='235', local_name='HundredGigE0/0/0/17', device_name='renc-data-sw')
        sliver_capacities = Capacities(bw=1000)

        stp_a.set_labels(sliver_labels)
        stp_a.set_capacities(sliver_capacities)

        #
        # STP_Z interface
        #
        stp_z = InterfaceSliver()
        stp_z.set_name('Interface2')
        stp_z.set_type(InterfaceType.ServicePort)

        sliver_labels = Labels(vlan='235', local_name='HundredGigE0/0/0/13', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1000)

        stp_z.set_labels(sliver_labels)
        stp_z.set_capacities(sliver_capacities)

        # create interface info object, add interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(stp_a)
        ifi.add_interface(stp_z)

        # All of this happens automagically in FIM
        sliver.interface_info = ifi
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L2PTP')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_L2STS(self):
        # create a NetworkService sliver for L2STS
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())

        #
        # create a network sliver for L2Bridge and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name - can we use the sliver name - only guaranteed unique in the slice
        sliver.set_name('L2STSServiceTest')
        sliver.set_type(ServiceType.L2STS)
        sliver.set_layer(NSLayer.L2)

        # ERO
        # first declare a path. Each path is a list of somethings. a2z and z2a maintained separately within Path
        ero_path = Path()

        ero_path.set_symmetric(["10.1.1.1", "10.1.1.2"])
        # default is loose ERO, set strict=True if want otherwise
        ero = ERO(strict=False)
        ero.set(ero_path)
        sliver.set_ero(ero)

        #
        # site A interfaces
        #
        stp_a1 = InterfaceSliver()
        stp_a1.set_name('Interface_A1')
        stp_a1.set_type(InterfaceType.ServicePort)
        sliver_labels = Labels(local_name='TwentyFiveGigE0/0/0/23/1', device_name='lbnl-data-sw')
        sliver_capacities = Capacities(bw=2000)
        # untagged w/o vlan label set
        stp_a1.set_labels(sliver_labels)
        stp_a1.set_capacities(sliver_capacities)

        #
        # site Z interfaces
        #
        stp_z1 = InterfaceSliver()
        stp_z1.set_name('Interface_Z1')
        stp_z1.set_type(InterfaceType.ServicePort)
        sliver_labels = Labels(vlan='235', local_name='HundredGigE0/0/0/13', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1000)
        stp_z1.set_labels(sliver_labels)
        stp_z1.set_capacities(sliver_capacities)

        stp_z2 = InterfaceSliver()
        stp_z2.set_name('Interface_Z2')
        stp_z2.set_type(InterfaceType.ServicePort)
        sliver_labels = Labels(local_name='TwentyFiveGigE0/0/0/23/2', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1000)
        # untagged w/o vlan label set
        stp_z2.set_labels(sliver_labels)
        stp_z2.set_capacities(sliver_capacities)

        # create interface info object, add interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(stp_a1)
        ifi.add_interface(stp_z1)
        ifi.add_interface(stp_z2)

        # All of this happens automagically in FIM
        sliver.interface_info = ifi
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L2STS')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_FABNetv4(self):
        # create a NetworkService sliver for FABNetv4
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3-UKY-IPv4')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.FABNetv4)
        sliver.set_layer(NSLayer.L3)

        # this is the gateway with the IP range picked for this sliver in this slice on this site
        # can also be specified with ipv6/ipv6_subnet and mac is optional for both.
        # Q: does that mean that the advertisement needs to maintain information about multiple
        # subnet, gateway and mac tuples for each site?
        sliver.set_gateway(Gateway(Labels(ipv4="10.128.128.254", ipv4_subnet="10.128.128.0/24")))

        #
        # create a small number of Interface slivers, set their properties and link to service
        #

        #
        # First interface - let's assume it is SR-IOV
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        # since this is SR-IOV, orchestrator picks VLAN for this function based on info in advertisement
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='121', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        # Q: who and how picks the VLAN in this case? I think we discussed having an advertised pool of 'Layer 3
        # vlans' which need to be kept track of and this would be one of them
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='1001', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_FABNetv4')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)


    def test_FABNetv4Ext(self):
        # create a NetworkService sliver for FABNetv4
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3-UKY-IPv4-Ext')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.FABNetv4Ext)
        sliver.set_layer(NSLayer.L3)

        # this is no need for gateway for ipv4 ext service

        # allow one IPv4 address for external access
        # sliver_labs = Labels(ipv4="10.128.128.123")
        # or allow for a list of IPv4 addresses
        sliver_labs = Labels(ipv4=["10.128.128.10", "10.128.128.20", "10.128.128.30"])
        sliver.set_labels(sliver_labs)

        #
        # create a small number of Interface slivers, set their properties and link to service
        #

        #
        # First interface - let's assume it is SR-IOV
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        # since this is SR-IOV, orchestrator picks VLAN for this function based on info in advertisement
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='121', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        # Q: who and how picks the VLAN in this case? I think we discussed having an advertised pool of 'Layer 3
        # vlans' which need to be kept track of and this would be one of them
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='1001', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_FABNetv4_Ext')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)


    def test_FABNetv4Ext_Modify(self):
        # create a NetworkService sliver for FABNetv4
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3-UKY-IPv4-Ext')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.FABNetv4Ext)
        sliver.set_layer(NSLayer.L3)

        # this is no need for gateway for ipv4 ext service

        # allow one IPv4 address for external access
        # sliver_labs = Labels(ipv4="10.128.128.123")
        # or allow for a list of IPv4 addresses
        #sliver_labs = Labels(ipv4=["10.128.128.10", "10.128.128.20", "10.128.128.30"])
        sliver_labs = Labels(ipv4=[])
        sliver.set_labels(sliver_labs)

        #
        # create a small number of Interface slivers, set their properties and link to service
        #

        #
        # First interface - let's assume it is SR-IOV
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        # since this is SR-IOV, orchestrator picks VLAN for this function based on info in advertisement
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='121', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        # Q: who and how picks the VLAN in this case? I think we discussed having an advertised pool of 'Layer 3
        # vlans' which need to be kept track of and this would be one of them
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='1001', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_FABNetv4_Ext')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        modified_sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        modified_sliver.set_name('L3-UKY-IPv4-Ext')  # must be the same name
        modified_sliver.set_type(ServiceType.FABNetv4Ext)
        modified_sliver.set_layer(NSLayer.L3)

        modified_sliver_labs = Labels(ipv4=["10.128.128.10", "10.128.128.20", "10.128.128.30"])
        modified_sliver.set_labels(modified_sliver_labs)
        modified_sliver.interface_info = ifi

        updated_unit.set_modified(modified=modified_sliver)
        r, updated_unit = handler.modify(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_MODIFY)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_FABNetv6(self):
        # create a NetworkService sliver for FABNetv6
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3-UKY-IPv6')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.FABNetv6)
        sliver.set_layer(NSLayer.L3)

        # this is the gateway with the IP range picked for this sliver in this slice on this site
        # can also be specified with ipv6/ipv6_subnet and mac is optional for both.
        # Q: does that mean that the advertisement needs to maintain information about multiple
        # subnet, gateway and mac tuples for each site?
        sliver.set_gateway(Gateway(Labels(ipv6="2602:FCFB:0001::1", ipv6_subnet="2602:FCFB:0001::/64")))

        #
        # create a small number of Interface slivers, set their properties and link to service
        #

        #
        # First interface - let's assume it is SR-IOV
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        # since this is SR-IOV, orchestrator picks VLAN for this function based on info in advertisement
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='121', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        # Q: who and how picks the VLAN in this case? I think we discussed having an advertised pool of 'Layer 3
        # vlans' which need to be kept track of and this would be one of them
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='1001', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_FABNetv6')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_FABNetv6Ext(self):
        # create a NetworkService sliver for FABNetv6
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3-UKY-IPv6')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.FABNetv6Ext)
        sliver.set_layer(NSLayer.L3)

        # allow one IPv6 address for external access
        # sliver_labs = Labels(ipv6="2602:FCFB:0001::10")
        # or allow for a list of IPv6 addresses
        sliver_labs = Labels(ipv6=["2602:FCFB:0001::10", "2602:FCFB:0001::20", "2602:FCFB:0001::30"])
        sliver.set_labels(sliver_labs)

        # this is the gateway with the IP range picked for this sliver in this slice on this site
        # can also be specified with ipv6/ipv6_subnet and mac is optional for both.
        # Q: does that mean that the advertisement needs to maintain information about multiple
        # subnet, gateway and mac tuples for each site?
        sliver.set_gateway(Gateway(Labels(ipv6="2602:FCFB:0001::1", ipv6_subnet="2602:FCFB:0001::/64")))

        #
        # create a small number of Interface slivers, set their properties and link to service
        #

        #
        # First interface - let's assume it is SR-IOV
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('Interface1')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)

        # since this is SR-IOV, orchestrator picks VLAN for this function based on info in advertisement
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='121', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')

        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=1)

        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_capacities(sliver_capacities)

        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('Interface2')
        isl2.set_type(InterfaceType.ServicePort)

        # Q: who and how picks the VLAN in this case? I think we discussed having an advertised pool of 'Layer 3
        # vlans' which need to be kept track of and this would be one of them
        # other information is done in the same way it is done for L2 services
        sliver_labels = Labels(vlan='1001', local_name='HundredGigE0/0/0/5', device_name='uky-data-sw')
        sliver_capacities = Capacities(bw=1)

        isl2.set_labels(sliver_labels)
        isl2.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_FABNetv6')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)


    def test_PortMirror(self):
        # create a NetworkService sliver for FABNetv6
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('PortMirror-RENC')
        sliver.set_type(ServiceType.PortMirror)
        sliver.set_layer(NSLayer.L2)

        # mirror_port is the name of the port being mirrored - actual name the way
        # service definition needs it. It comes directly from ASM network service sliver
        # whatever the right name is - user must to know it when creating a slice
        sliver.mirror_port = "HundredGigE0/0/0/17"
        # Optional: set the vlan tag
        sliver.mirror_vlan = "100"
        # direction also comes from ASM network service sliver
        sliver.mirror_direction = MirrorDirection.Both

        #
        # interface to which the mirrored traffic is directed to
        # it is in the slice (ASM) model, the same way all other interfaces for network services are.
        #
        stp_to = InterfaceSliver()
        stp_to.set_name('Interface_To_Which_We_Send_Mirrored_Traffic')
        stp_to.set_type(InterfaceType.ServicePort)
        sliver_labels = Labels(local_name='HundredGigE0/0/0/19',
                               device_name='renc-data-sw',
                               # Optional: set VLAN tag
                                vlan="100"
                               )
        sliver_capacities = Capacities(bw=2000)
        stp_to.set_labels(sliver_labels)
        stp_to.set_capacities(sliver_capacities)

        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(stp_to)

        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_PortMirror')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)

        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_L3VPN(self):
        # create a NetworkService sliver for FABNetv6
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for L3VPN and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3VPN-IPv4-Test')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.L3VPN)
        sliver.set_layer(NSLayer.L3)
        sliver.set_labels(Labels(asn='398900'))

        # route by BGP peer
        stp1 = InterfaceSliver()
        stp1.set_name('Interface1')
        stp1.set_type(InterfaceType.ServicePort)
        interface_labels = Labels(vlan='1001', local_name='TwentyFiveGigE0/0/0/23/1', device_name='lbnl-data-sw', ipv4_subnet='192.168.10.1/24')
        peering_labels = Labels(ipv4_subnet='192.168.10.2/24', asn='654321',  bgp_key='somesecret')
        sliver_capacities = Capacities(bw=1)
        stp1.set_labels(interface_labels)
        stp1.set_peer_labels(peering_labels)
        stp1.set_capacities(sliver_capacities)

        # route by direct connect ++ (additional direct connecton on same site)
        stp1a = InterfaceSliver()
        stp1a.set_name('Interface1a')
        stp1a.set_type(InterfaceType.ServicePort)
        interface_labels = Labels(vlan='1002', local_name='TwentyFiveGigE0/0/0/23/1', device_name='lbnl-data-sw', ipv4_subnet='192.168.30.1/24')
        sliver_capacities = Capacities(bw=1)
        stp1a.set_labels(interface_labels)
        stp1a.set_capacities(sliver_capacities)

        # route by BGP peer ++ (additional bgp connecton on same site)
        stp1b = InterfaceSliver()
        stp1b.set_name('Interface1b')
        stp1b.set_type(InterfaceType.ServicePort)
        interface_labels = Labels(vlan='2001', local_name='TwentyFiveGigE0/0/0/23/1', device_name='lbnl-data-sw', ipv4_subnet='192.168.20.1/24')
        peering_labels = Labels(ipv4_subnet='192.168.20.2/24', asn='654322',  bgp_key='somesecret')
        sliver_capacities = Capacities(bw=1)
        stp1b.set_labels(interface_labels)
        stp1b.set_peer_labels(peering_labels)
        stp1b.set_capacities(sliver_capacities)


        # route by direct connect
        stp2 = InterfaceSliver()
        stp2.set_name('Interface2')
        stp2.set_type(InterfaceType.ServicePort)
        interface_labels = Labels(vlan='1002', local_name='TwentyFiveGigE0/0/0/23/1', device_name='uky-data-sw',  ipv4_subnet='192.168.20.1/24')
        sliver_capacities = Capacities(bw=1)
        stp2.set_labels(interface_labels)
        stp2.set_capacities(sliver_capacities)

        # route by direct connect
        stp3 = InterfaceSliver()
        stp3.set_name('Interface3')
        stp3.set_type(InterfaceType.ServicePort)
        interface_labels = Labels(vlan='1003', local_name='TwentyFiveGigE0/0/0/23/1', device_name='uky-data-sw',  ipv4_subnet='192.168.20.1/24')
        sliver_capacities = Capacities(bw=1)
        stp3.set_labels(interface_labels)
        stp3.set_capacities(sliver_capacities)

        ifi = InterfaceInfo()
        sliver.interface_info = ifi
        ifi.add_interface(stp1)
        ifi.add_interface(stp1a)
        ifi.add_interface(stp1b)
        ifi.add_interface(stp2)
        ifi.add_interface(stp3)

        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L3VPN')
        self.unit = Unit(rid=ID(uid=str(uid)))
        self.unit.set_sliver(sliver=sliver)

        #
        # create a service (create needs to parse out sliver information
        # into exact parameters the service ansible script needs)
        #
        r, updated_unit = handler.create(unit=self.unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

        time.sleep(30)
        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)

    def test_CleanRestart(self):
        # create a NetworkService sliver for FABNetv6
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/net_handler_config.yml'}

        handler = NetHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        r = handler.clean_restart()
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CLEAN_RESTART)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_EXCEPTION)
