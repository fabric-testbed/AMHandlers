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
from fabric_am.handlers.al2s_handler import Al2sHandler
from fabric_am.util.am_constants import AmConstants

class TestAl2sHandler(unittest.TestCase):
    logger = logging.getLogger(__name__)
    log_format = \
        '%(asctime)s - %(name)s - {%(filename)s:%(lineno)d} - [%(threadName)s] - %(levelname)s - %(message)s'
    logging.basicConfig(handlers=[logging.StreamHandler()], format=log_format, force=True)

    def setUp(self) -> None:
        self.unit = Unit(rid=ID(uid="rid-1"))
    
    
    # def test_L2Cloud(self):
    #     # create a NetworkService sliver for L2Cloud
    #     prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/oess_handler_config.yml'}
    #
    #     handler = Al2sHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
    #
    #     #
    #     # create a network sliver for L2Cloud and its interfaces
    #     #
    #     sliver = NetworkServiceSliver();
    #     # service name - can we use the sliver name - only guaranteed unique in the slice
    #     sliver.set_name('al2s-circuit-cloud')
    #     sliver.set_type(ServiceType.L2PTP)
    #     sliver.set_layer(NSLayer.L2)
    #
    #     #
    #     # endpoint_a interface
    #     #
    #     endpoint_a = InterfaceSliver()
    #     endpoint_a.set_name('Interface1')
    #     endpoint_a.set_type(InterfaceType.ServicePort)
    #
    #     sliver_labels = Labels();
    #     sliver_capacities = Capacities()
    #
    #     sliver_labels._set_fields(vlan='2')
    #     sliver_labels._set_fields(local_name='HundredGigE0/0/0/7')
    #     sliver_labels._set_fields(device_name='agg4.ashb.net.internet2.edu')
    #     sliver_labels._set_fields(account_id='296256999979')
    #
    #     sliver_capacities._set_fields(bw=1, mtu=9001)
    #
    #     endpoint_a.set_labels(sliver_labels)
    #     endpoint_a.set_capacities(sliver_capacities)
    #
    #     #
    #     # endpoint_b interface
    #     #
    #     endpoint_b = InterfaceSliver()
    #     endpoint_b.set_name('Interface2')
    #     endpoint_b.set_type(InterfaceType.ServicePort)
    #
    #     # sliver_labels = OessLabels()
    #     # sliver_capacities = OessCapacities()
    #     sliver_labels = Labels()
    #     sliver_capacities = Capacities()
    #
    #     sliver_labels._set_fields(vlan='2')
    #     sliver_labels._set_fields(local_name='TenGigE0/0/0/12/2')
    #     sliver_labels._set_fields(device_name='agg4.sanj.net.internet2.edu')
    #     sliver_labels._set_fields(account_id='296256999979')
    #
    #     sliver_capacities._set_fields(bw=1, mtu=9001)
    #
    #     endpoint_b.set_labels(sliver_labels)
    #     endpoint_b.set_capacities(sliver_capacities)
    #
    #     # create interface info object, add interfaces to it
    #     ifi = InterfaceInfo()
    #     ifi.add_interface(endpoint_a)
    #     ifi.add_interface(endpoint_b)
    #     #
    #     # # All of this happens automagically in FIM
    #     sliver.interface_info = ifi
    #     uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L2Cloud')
    #     self.unit = Unit(rid=ID(uid=str(uid)))
    #     self.unit.set_sliver(sliver=sliver)
    #
    #     #
    #     # create a service
    #     #
    #     r, updated_unit = handler.create(unit=self.unit)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
    #     self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
    #
    #     time.sleep(30)
    #
    #     #
    #     # delete - need to make sure the updated unit has the right info to delete the service
    #     #
    #     r, updated_unit = handler.delete(updated_unit)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
    #     self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
    #
    #
    # def test_L3Cloud(self):
    #     # create a NetworkService sliver for FABNetv4
    #     prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/oess_handler_config.yml'}
    #
    #     handler = Al2sHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
    #     #
    #     # create a network sliver for FABNetv4 and its interfaces
    #     #
    #     sliver = NetworkServiceSliver()
    #     # service name (set by user) - only guaranteed unique within a slice
    #     sliver.set_name('L3VPN-Cloud')
    #     # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
    #     # e.g. concatenate name + res id (or another unique id)
    #     # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
    #     # per @xiyang he uses unit id for service name so this is not needed.
    #     sliver.set_type(ServiceType.L3VPN)
    #     sliver.set_layer(NSLayer.L3)
    #     # the ASN of *this* service
    #     sliver.set_labels(Labels(asn='55038', local_name='al2s_l3_directconn_test'))
    #
    #     # this is the gateway with the IP range picked for this sliver in this slice on this site
    #     # can also be specified with ipv6/ipv6_subnet and mac is optional for both.
    #     # Q: does that mean that the advertisement needs to maintain information about multiple
    #     # subnet, gateway and mac tuples for each site?
    #     # sliver.set_gateway(Gateway(Labels(ipv4="10.128.128.254", ipv4_subnet="10.128.128.0/24")))
    #
    #     #
    #     # create a small number of Interface slivers, set their properties and link to service
    #     #
    #
    #     #
    #     # First interface
    #     #
    #     isl1 = InterfaceSliver()
    #     # the name is normally set by FIM as '-' concatenation of service name
    #     isl1.set_name('Interface1')
    #     # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
    #     # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
    #     # name and peer interface sliver name.
    #     isl1.set_type(InterfaceType.ServicePort)
    #
    #     ####################
    #     #   Scenario #1
    #     ####################
    #     # sliver_labels = Labels(ipv4_subnet='192.168.1.2/30',
    #     #                        vlan='2', 
    #     #                        local_name='HundredGigE0/0/0/7',
    #     #                        device_name='agg4.ashb.net.internet2.edu')
    #     # sliver_peer_labels = Labels(ipv4_subnet='192.168.1.1/30', asn='64512',
    #     #                             bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx', account_id='296256999979')
    #     #
    #     # # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
    #     # sliver_capacities = Capacities(bw=1, mtu=9001)
    #
    #     ####################
    #     #   Scenario #2
    #     ####################
    #     # sliver_labels = Labels(ipv4_subnet='192.168.30.1/24',
    #     #                        vlan='203', 
    #     #                        local_name='agg4.dall3.net.internet2.edu:TenGigE0/0/0/14/2',
    #     #                        device_name='AL2S')
    #     # sliver_peer_labels = Labels(ipv4_subnet='192.168.30.2/24', asn='64512',
    #     #                             bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx', account_id='296256999979')
    #     #
    #     # # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
    #     # sliver_capacities = Capacities(bw=10)
    #
    #     ####################
    #     #   Scenario #3
    #     ####################
    #     # sliver_labels = Labels(ipv4_subnet='192.168.50.1/24',
    #     #                        vlan='852', 
    #     #                        local_name='HundredGigE0/0/0/24',
    #     #                        device_name='core1.loui.net.internet2.edu')
    #     # sliver_peer_labels = Labels(ipv4_subnet='192.168.50.2/24', asn='398900',
    #     #                             bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx', account_id='296256999979')
    #     #
    #     # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
    #     # sliver_capacities = Capacities(bw=0)
    #
    #     ####################
    #     #   Scenario #4
    #     ####################
    #     sliver_labels = Labels(ipv4_subnet='192.168.50.1/24',
    #                            vlan='852', 
    #                            local_name='HundredGigE0/0/0/24',
    #                            device_name='core1.loui.net.internet2.edu',
    #                            bgp_key ='secret')
    #     sliver_peer_labels = Labels(ipv4_subnet='192.168.50.2/24', asn='398900',
    #                                 bgp_key='secret')
    #
    #     # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
    #     sliver_capacities = Capacities(bw=0)
    #
    #     ####################
    #     #   End of Scenario
    #     ####################
    #
    #     # assign labels and capacities
    #     isl1.set_labels(sliver_labels)
    #     isl1.set_peer_labels(sliver_peer_labels)
    #     isl1.set_capacities(sliver_capacities)
    #
    #     #
    #     # Second interface (let's assume this is a dedicated card)
    #     #
    #     isl2 = InterfaceSliver()
    #     isl2.set_name('Interface2')
    #     isl2.set_type(InterfaceType.ServicePort)
    #
    #     ####################
    #     #   Scenario #1
    #     ####################
    #     # sliver_labels = Labels(ipv4_subnet='192.168.2.2/30',
    #     #                        vlan='2', 
    #     #                        local_name='TenGigE0/0/0/12/2',
    #     #                        device_name='agg4.sanj.net.internet2.edu')
    #     # sliver_peer_labels = Labels(ipv4_subnet='192.168.2.1/30', asn='64512',
    #     #                             bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx', account_id='296256999979')
    #     #
    #     # # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
    #     # sliver_capacities = Capacities(bw=1, mtu=9001)
    #
    #     ####################
    #     #   Scenario #2
    #     ####################
    #     sliver_labels = Labels(ipv4_subnet='192.168.30.1/24',
    #                            vlan='204', 
    #                            local_name='HundredGigE0/0/0/7',
    #                            device_name='agg3.ashb.net.internet2.edu')
    #     sliver_peer_labels = Labels(ipv4_subnet='192.168.30.2/24', asn='64512',
    #                                 bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx',
    #                                 account_id='296256999979')
    #
    #     # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
    #     sliver_capacities = Capacities(bw=1)
    #
    #     ####################
    #     #   End of Scenario
    #     ####################
    #
    #     isl2.set_labels(sliver_labels)
    #     isl2.set_peer_labels(sliver_peer_labels)
    #     isl2.set_capacities(sliver_capacities)
    #
    #     # create interface info object, add populated interfaces to it
    #     ifi = InterfaceInfo()
    #     ifi.add_interface(isl1)
    #     ifi.add_interface(isl2)
    #
    #     # add interface info object to sliver. All of this happens automagically normally
    #     sliver.interface_info = ifi
    #
    #     # set a fake unit reservation
    #     uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L3Cloud')
    #     self.unit = Unit(rid=ID(uid=str(uid)))
    #     self.unit.set_sliver(sliver=sliver)
    #
    #     #
    #     # create a service (create needs to parse out sliver information
    #     # into exact parameters the service ansible script needs)
    #     #
    #     r, updated_unit = handler.create(unit=self.unit)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_CREATE)
    #     self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
    #
    #     time.sleep(30)
    #
    #     #
    #     # delete - need to make sure the updated unit has the right info to delete the service
    #     #
    #     r, updated_unit = handler.delete(updated_unit)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
    #     self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
    #     self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
    

    
    def test_L3Cloud_GCP(self):
        # create a NetworkService sliver for FABNetv4
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/al2s_handler_config.yml'}
    
        handler = Al2sHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3VPN-Cloud')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.L3VPN)
        sliver.set_layer(NSLayer.L3)
        # the ASN of *this* service
        sliver.set_labels(Labels(asn='55038', local_name='al2s_l3_gcp_interconn_test'))
    
        # this is the gateway with the IP range picked for this sliver in this slice on this site
        # can also be specified with ipv6/ipv6_subnet and mac is optional for both.
        # Q: does that mean that the advertisement needs to maintain information about multiple
        # subnet, gateway and mac tuples for each site?
        # sliver.set_gateway(Gateway(Labels(ipv4="10.128.128.254", ipv4_subnet="10.128.128.0/24")))
    
        #
        # create a small number of Interface slivers, set their properties and link to service
        #
    
        #
        # First interface
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('Google Cloud Platform')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)
    
        sliver_labels = Labels(ipv4_subnet='192.168.100.1/30',
                               vlan='100', 
                               local_name='Bundle-Ether5',
                               device_name='agg3.ashb')
        sliver_peer_labels = Labels(ipv4_subnet='192.168.100.2/30', asn='16550',
                                    bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx',
                                    account_id='ae835d22-b10c-4d40-bd98-0dbc4259684d/us-east4/2',
                                    local_name='Google Cloud Platform')
    
        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=50, mtu=9001)
    
        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_peer_labels(sliver_peer_labels)
        isl1.set_capacities(sliver_capacities)
    
        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('FABRIC')
        isl2.set_type(InterfaceType.ServicePort)
    
        sliver_labels = Labels(ipv4_subnet='192.168.100.2/30',
                               vlan='100', 
                               local_name='HundredGigE0/0/0/27',
                               device_name='core1.star')
        sliver_peer_labels = Labels(ipv4_subnet='192.168.100.1/30', asn='64512',
                                    bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx',
                                    local_name='FABRIC')
    
        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=0, mtu=9000)
    
        isl2.set_labels(sliver_labels)
        isl2.set_peer_labels(sliver_peer_labels)
        isl2.set_capacities(sliver_capacities)
    
        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)
    
        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi
    
        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L3Cloud')
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
    
        time.sleep(60)
    
        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
    

    
    def test_L3Cloud_AWS(self):
        # create a NetworkService sliver for FABNetv4
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: '../config/al2s_handler_config.yml'}
    
        handler = Al2sHandler(logger=self.logger, properties=prop, process_lock=threading.Lock())
        #
        # create a network sliver for FABNetv4 and its interfaces
        #
        sliver = NetworkServiceSliver()
        # service name (set by user) - only guaranteed unique within a slice
        sliver.set_name('L3VPN-Cloud')
        # if service name global uniqueness is a requirement use Labels.local_name for that (optional)
        # e.g. concatenate name + res id (or another unique id)
        # sliver.set_labels(Labels().set_fields(local_name='test-l2bridge-shortname'))
        # per @xiyang he uses unit id for service name so this is not needed.
        sliver.set_type(ServiceType.L3VPN)
        sliver.set_layer(NSLayer.L3)
        # the ASN of *this* service
        sliver.set_labels(Labels(asn='55038', local_name='al2s_l3_aws_interconn_test1'))
    
        # this is the gateway with the IP range picked for this sliver in this slice on this site
        # can also be specified with ipv6/ipv6_subnet and mac is optional for both.
        # Q: does that mean that the advertisement needs to maintain information about multiple
        # subnet, gateway and mac tuples for each site?
        # sliver.set_gateway(Gateway(Labels(ipv4="10.128.128.254", ipv4_subnet="10.128.128.0/24")))
    
        #
        # create a small number of Interface slivers, set their properties and link to service
        #
    
        #
        # First interface
        #
        isl1 = InterfaceSliver()
        # the name is normally set by FIM as '-' concatenation of service name
        isl1.set_name('AWS')
        # this will be a ServicePort in the network service sliver. It is created by FIM automatically when
        # the user adds a NetworkService to the ASM. The name is set by the FIM as '-' concatenation of service
        # name and peer interface sliver name.
        isl1.set_type(InterfaceType.ServicePort)
    
        sliver_labels = Labels(ipv4_subnet='192.168.5.2/30',
                               vlan='501', 
                               local_name='TenGigE0/0/0/12/2',
                               device_name='agg4.sanj')
        sliver_peer_labels = Labels(ipv4_subnet='192.168.5.1/30', asn='64512',
                                    bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx', account_id='296256999979',
                                    local_name='AWS')
    
        # capacities (bw in Mbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=50, mtu=9001)
    
        # assign labels and capacities
        isl1.set_labels(sliver_labels)
        isl1.set_peer_labels(sliver_peer_labels)
        isl1.set_capacities(sliver_capacities)
    
        #
        # Second interface (let's assume this is a dedicated card)
        #
        isl2 = InterfaceSliver()
        isl2.set_name('FABRIC')
        isl2.set_type(InterfaceType.ServicePort)
    
        sliver_labels = Labels(ipv4_subnet='192.168.100.2/30',
                               vlan='100', 
                               local_name='HundredGigE0/0/0/27',
                               device_name='core1.star')
        sliver_peer_labels = Labels(ipv4_subnet='192.168.100.1/30', asn='64512',
                                    bgp_key='0xzsEwC7xk6c1fK_h.xHyAdx',
                                    local_name='FABRIC')
    
        # capacities (bw in Gbps, burst size is in Mbytes) source: (b)
        sliver_capacities = Capacities(bw=0, mtu=9000)
    
        isl2.set_labels(sliver_labels)
        isl2.set_peer_labels(sliver_peer_labels)
        isl2.set_capacities(sliver_capacities)
    
        # create interface info object, add populated interfaces to it
        ifi = InterfaceInfo()
        ifi.add_interface(isl1)
        ifi.add_interface(isl2)
    
        # add interface info object to sliver. All of this happens automagically normally
        sliver.interface_info = ifi
    
        # set a fake unit reservation
        uid = uuid.uuid3(uuid.NAMESPACE_DNS, 'test_L3Cloud')
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
    
        time.sleep(60)
    
        #
        # delete - need to make sure the updated unit has the right info to delete the service
        #
        r, updated_unit = handler.delete(updated_unit)
        self.assertEqual(r[Constants.PROPERTY_TARGET_NAME], Constants.TARGET_DELETE)
        self.assertEqual(r[Constants.PROPERTY_ACTION_SEQUENCE_NUMBER], 0)
        self.assertEqual(r[Constants.PROPERTY_TARGET_RESULT_CODE], Constants.RESULT_CODE_OK)
    

