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


class AmConstants:
    CONFIG_PROPERTIES_FILE = "config.properties.file"

    EC2_AVAILABILITY_ZONE = "availability_zone"

    PLAYBOOK_SECTION = "playbooks"
    PB_LOCATION = "location"
    PB_INVENTORY = "inventory_location"
    PB_HOSTNAME_SUFFIX = "hostname_suffix"

    VM_PROV_OP = "vm_prov_op"
    VM_PROV_OP_CREATE = "create"
    VM_PROV_OP_DELETE = "delete"
    VM_PROV_OP_ATTACH_FIP = "attach_fip"
    VM_PROV_OP_GET = "get"

    SERVER = "server"
    SERVER_VM_STATE = "vm_state"
    SERVER_INSTANCE_NAME = "instance_name"
    SERVER_ACCESS_IPV4 = "accessIPv4"

    FLOATING_IP = "floating_ip"
    FLOATING_IP_ADDRESS = "floating_ip_address"
    FLOATING_IP_PROPERTIES = "properties"
    FLOATING_IP_PORT_DETAILS = "port_details"
    FLOATING_IP_MAC_ADDRESS = "mac_address"

    KVM_GUEST_NAME = "kvmguest_name"
    PCI_ADDRESS = "pcidevice_address"
    PCI_DOMAIN = "domain"
    PCI_BUS = "bus"
    PCI_SLOT = "slot"
    PCI_FUNCTION = "function"
    PCI_OPERATION = "pci_prov_op"
    PCI_PROV_ATTACH = "attach"
    PCI_PROV_DETACH = "detach"
    PCI_PROV_DEVICE = "device"
    WORKER_NODE_NAME = "worker_node_name"

    ANSIBLE_FACTS = "ansible_facts"

    VM_NAME = "vmname"
    FLAVOR = "flavor"
    IMAGE = "image"
    HOSTNAME = "hostname"
    SSH_KEY = "sshkey"
    DEFAULT_USER = "default_user"

    CENTOS_DEFAULT_USER = "centos"
    UBUNTU_DEFAULT_USER = "ubuntu"
    ROOT_USER = "root"
