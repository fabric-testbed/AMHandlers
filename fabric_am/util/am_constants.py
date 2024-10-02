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

    ANSIBLE_SECTION = "ansible"
    ANSIBLE_PYTHON_INTERPRETER = "ansible_python_interpreter"

    RUNTIME_SECTION = "runtime"
    RT_DISABLE_FIP = "disable_fip"
    RT_SSH_RETRIES = "ssh_retries"
    RT_DELETE_RETRIES = "delete_retries"
    RT_IMAGES = "images"

    PLAYBOOK_SECTION = "playbooks"
    PB_LOCATION = "location"
    PB_POST_BOOT = "post_boot"
    PB_INVENTORY = "inventory_location"
    PB_HOSTNAME_SUFFIX = "hostname_suffix"
    PB_CLEANUP = "cleanup"
    PB_CONFIG = "config"
    PB_SSH_KEYS = "ssh_keys"

    CLEAN_ALL = "ALL"

    OPERATION = "operation"
    OP_ATTACH = "attach"
    OP_DETACH = "detach"
    OP_CREATE = "create"
    OP_DELETE = "delete"
    OP_IS_DELETED = "is_deleted"
    OP_LIST = "list"
    OP_DELETE_ALL = "delete_all"
    OP_ATTACH_FIP = "attach_fip"
    OP_GET = "get"
    OP_REBOOT = "reboot"
    OP_STOP = "stop"
    OP_START = "start"
    OP_CPUINFO = "cpuinfo"
    OP_NUMAINFO = "numainfo"
    OP_CPUPIN = "cpupin"
    OP_NUMATUNE = "numatune"
    OP_MOUNT = "mount"
    OP_ADDKEY = "addkey"
    OP_REMOVEKEY = "removekey"
    OP_CONFIG = "config"
    OP_POST_REBOOT = "post-reboot"

    PORT_NAME = "portname"
    NETWORK_NAME = "networkname"
    NETWORK_NAME_PREFIX = "network_name_prefix"
    IP_ADDRESS = "ip_address"
    VM_EXISTS = "vm_exists"

    VOL_NAME = "vol_name"
    ATTACHMENTS = "attachments"

    SERVER = "server"
    SERVER_VM_STATE = "vm_state"
    SERVER_INSTANCE_NAME = "instance_name"
    SERVER_ACCESS_IPV4 = "accessIPv4"
    SERVER_ACCESS_IPV6 = "accessIPv6"
    OS_SERVERS = "openstack_servers"

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
    PCI_BDF = "bdf"
    PCI_DEVICE_NUMBER = "pci_device_number"
    PCI_FUNCTION = "function"
    DEVICE = "device"
    NUM_PCI = "num_pci"
    WORKER_NODE_NAME = "worker_node_name"
    INTERFACE_NAME = "if_name"
    USB_REQUIRED = "usb_required"

    ANSIBLE_FACTS = "ansible_facts"
    COMBINED_FACTS = "combined_facts"

    VM_NAME = "vmname"
    FLAVOR = "flavor"
    IMAGE = "image"
    HOSTNAME = "hostname"
    SSH_KEY = "sshkey"
    SDE_FILE_NAME = "sde_file_name"
    DEFAULT_USER = "default_user"
    INIT_SCRIPT = "init_script"
    USER_DATA = "user_data"
    USER = "user"
    KEYS = "keys"
    P4 = "P4"

    ROOT_USER = "root"

    VLAN = "vlan"
    MAC = "mac"
    IPV4_ADDRESS = "ipv4_address"
    IPV4_PREFIX = "ipv4_prefix"
    IPV6_ADDRESS = "ipv6_address"
    IPV6_PREFIX = "ipv6_prefix"
    ADDRESS_LIST = "address_list"
    ADMIN_SSH_KEY = "admin_ssh_key"

    NETWORK_SECTION = "network"
    NET_BORDER_ROUTERS = "border_routers"
    ROCKY = "rocky"
    CENTOS = "centos"
    UBUNTU = "ubuntu"
    DEBIAN = "debian"
    FEDORA = "fedora"

    VM = "vm"
    HOST = "host"
    VCPU_CPU_MAP = "vcpu_cpu_map"
    NODE_SET = "node_set"

    MAX_FLAVOR = "max_flavor"
