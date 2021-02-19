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
    CONFIG_PROPERTIES_FILE = 'config.properties.file'

    EC2_AVAILABILITY_ZONE = 'availability_zone'

    PLAYBOOK_SECTION = 'playbooks'
    PB_VM_PROVISIONING = 'vm_provisioning'
    PB_LOCATION = "location"
    PB_INVENTORY = "inventory_location"

    VM_PROV_OP = "vm_prov_op"
    VM_PROV_OP_CREATE = "create"
    VM_PROV_OP_DELETE = "delete"
    VM_PROV_OP_ATTACH_FIP = "attach_fip"

    SERVER = 'server'
    SERVER_VM_STATE = 'vm_state'
    SERVER_INSTANCE_NAME = 'instance_name'
    SERVER_ACCESS_IPv4 = 'accessIPv4'
