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
import re
import time
import traceback
from typing import List, Dict, Any

import paramiko

from fabric_am.util.am_constants import AmConstants
from fabric_am.util.ansible_helper import AnsibleHelper


class Utils:
    """
    Helper class to basic support functions
    """
    @staticmethod
    def parse_vcpuinfo(*, vcpuinfo_output: List[str]) -> List[Dict[str, str]]:
        """
        Parse VcpuInfo and transform it to JSON
        @param vcpuinfo_output: vcpuinfo command output
        """
        '''
        Vcpuinfo Output looks like:
            cpu_info = ["VCPU:           0",
                "CPU:            27",
                "State:          running",
                "CPU time:       3.1s",
                "CPU Affinity:   yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy", 
                "", 
                "VCPU:           1", 
                "CPU:            9", 
                "State:          running", 
                "CPU Affinity:   yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy"
                ]
        '''
        result = []
        data = {}
        for line in vcpuinfo_output:
            if line == "":
                result.append(data)
                data.clear()
            else:
                # Split the line at the first occurrence of ':' character
                key, value = line.split(':', 1)
                # Remove leading/trailing spaces from key and value
                key = key.strip()
                value = value.strip()
                # Add the key-value pair to the data dictionary
                data[key] = value
        return result

    @staticmethod
    def parse_lscpu(*, lscpu_output: List[str]) -> List[Dict[str, str]]:
        """
        Parse Lscpu and transform it to JSON
        @param lscpu_output: lscpu command output
        """
        '''
        ['CPU NODE', '0   0', '1   0', '2   0', '3   0', '4   0', '5   0', '6   0', '7   0', '8   1', '9   1', '10  1',
         '11  1', '12  1', '13  1', '14  1', '15  1', '16  2', '17  2', '18  2', '19  2', '20  2', '21  2', '22  2', 
         '23  2', '24  3', '25  3', '26  3', '27  3', '28  3', '29  3', '30  3', '31  3', '32  4', '33  4', '34  4', 
         '35  4', '36  4', '37  4', '38  4', '39  4', '40  5', '41  5', '42  5', '43  5', '44  5', '45  5', '46  5', 
         '47  5', '48  6', '49  6', '50  6', '51  6', '52  6', '53  6', '54  6', '55  6', '56  7', '57  7', '58  7', 
         '59  7', '60  7', '61  7', '62  7', '63  7', '64  0', '65  0', '66  0', '67  0', '68  0', '69  0', '70  0', 
         '71  0', '72  1', '73  1', '74  1', '75  1', '76  1', '77  1', '78  1', '79  1', '80  2', '81  2', '82  2', 
         '83  2', '84  2', '85  2', '86  2', '87  2', '88  3', '89  3', '90  3', '91  3', '92  3', '93  3', '94  3', 
         '95  3', '96  4', '97  4', '98  4', '99  4', '100 4', '101 4', '102 4', '103 4', '104 5', '105 5', '106 5', 
         '107 5', '108 5', '109 5', '110 5', '111 5', '112 6', '113 6', '114 6', '115 6', '116 6', '117 6', '118 6', 
         '119 6', '120 7', '121 7', '122 7', '123 7', '124 7', '125 7', '126 7', '127 7']
        '''
        result = []
        for line in lscpu_output:
            if line == "" or "CPU" in line:
                continue
            else:
                data = {}
                # Split the line at the first occurrence of ' ' character
                cpu, node = line.split(' ', 1)
                # Remove leading/trailing spaces from key and value
                cpu = cpu.strip()
                node = node.strip()
                # Add the key-value pair to the data dictionary
                data['cpu'] = cpu
                data['node'] = node
                result.append(data)
        return result

    @staticmethod
    def parse_numastat(*, numastat_output: str) -> Dict[Any, dict]:
        """
        Parse numa stat output and convert it to JSON
        @param numastat_output: Numa State output
        """
        '''
        Numa Stat Output looks like:
        numa_info = ["",
                 "Per-node process memory usage (in MBs) for PID 2676600 (qemu-kvm)",
                 "         Node 0 Node 1 Total",
                 "         ------ ------ -----",
                 "Private   17531  15284 32815",
                 "Heap          0      6     6",
                 "Stack         0      0     0",
                 "Huge          0      0     0",
                 "-------  ------ ------ -----",
                 "Total     17531  15290 32821"
                 ]
        '''
        result = {}
        keys = []
        for line in numastat_output:
            # Ignore empty lines and headers
            if line == "" or "Per-node" in line or "--" in line:
                continue
            # Extract Numa Nodes
            elif "Node" in line:
                line = line.strip()
                regex_pattern = r'\b(\w+\s+\d+)\b|\b(\w+)\b'
                matches = re.findall(regex_pattern, line)
                # Flatten the list of tuples and remove empty strings
                keys = [match[0] or match[1] for match in matches if match[0] or match[1]]

                # Add empty dictionary for each Node and Total
                for x in keys:
                    result[x] = {}
            else:
                # Extract Different types of Memory for each Node and Total
                regex_pattern = r'(\b\w+\b)|(\b\d+\b)'
                matches = re.findall(regex_pattern, line)
                # Flatten the list of tuples and remove empty strings
                memory_values = [match[0] or match[1] for match in matches if match[0] or match[1]]
                # Index 0 contains the type of Memory and is used as a key
                idx = 1
                for x in keys:
                    result[x][memory_values[0]] = memory_values[idx]
                    idx += 1

        return result

    @staticmethod
    def parse_numactl(*, numactl_output: str) -> Dict[Any, dict]:
        """
        Parse numa stat output and convert it to JSON
        @param numastat_output: Numa State output
        """
        '''
        Numa Stat Output looks like:
        numa_info = [
                     'available: 8 nodes (0-7)', 
                     'node 0 cpus: 0 1 2 3 4 5 6 7 64 65 66 67 68 69 70 71', 
                     'node 0 size: 63798 MB', 
                     'node 0 free: 59660 MB', 
                     'node 1 cpus: 8 9 10 11 12 13 14 15 72 73 74 75 76 77 78 79', 
                     'node 1 size: 64507 MB', 
                     'node 1 free: 63042 MB', 
                     'node 2 cpus: 16 17 18 19 20 21 22 23 80 81 82 83 84 85 86 87', 
                     'node 2 size: 64507 MB', 
                     'node 2 free: 63911 MB', 
                     'node 3 cpus: 24 25 26 27 28 29 30 31 88 89 90 91 92 93 94 95', 
                     'node 3 size: 64458 MB', 
                     'node 3 free: 30882 MB', 
                     'node 4 cpus: 32 33 34 35 36 37 38 39 96 97 98 99 100 101 102 103', 
                     'node 4 size: 64507 MB', 
                     'node 4 free: 64069 MB', 
                     'node 5 cpus: 40 41 42 43 44 45 46 47 104 105 106 107 108 109 110 111', 
                     'node 5 size: 64507 MB', 
                     'node 5 free: 63966 MB', 
                     'node 6 cpus: 48 49 50 51 52 53 54 55 112 113 114 115 116 117 118 119', 
                     'node 6 size: 64507 MB', 
                     'node 6 free: 61214 MB', 
                     'node 7 cpus: 56 57 58 59 60 61 62 63 120 121 122 123 124 125 126 127', 
                     'node 7 size: 64506 MB', 
                     'node 7 free: 63869 MB', 
                     'node distances:', 
                     'node   0   1   2   3   4   5   6   7 ', 
                     '  0:  10  12  12  12  32  32  32  32 ', 
                     '  1:  12  10  12  12  32  32  32  32 ', 
                     '  2:  12  12  10  12  32  32  32  32 ', 
                     '  3:  12  12  12  10  32  32  32  32 ', 
                     '  4:  32  32  32  32  10  12  12  12 ', 
                     '  5:  32  32  32  32  12  10  12  12 ', 
                     '  6:  32  32  32  32  12  12  10  12 ', 
                     '  7:  32  32  32  32  12  12  12  10 '
                     ]
        '''
        result = {}
        data = {}
        for line in numactl_output:
            # Ignore empty lines and headers
            if line == "":
                continue
            elif "available" in line:
                key, value = line.split(':')
                key = key.strip()
                value = value.strip()
                result[key] = value
            elif "node distances" in line:
                break
            else:
                key, value = line.split(':')
                key = key.strip()
                value = value.strip()
                regex_pattern = r"^(.*?)\s+(\w+)$"
                matches = re.match(regex_pattern, key)
                if matches:
                    node = matches.group(1)
                    key = matches.group(2)
                    if node not in result:
                        result[node] = {}
                    result[node][key] = value
                else:
                    result[key] = value
        return result

    @staticmethod
    def execute_command(*, mgmt_ip: str, user: str, command: str, logger: logging.Logger,
                        timeout: int = 60, retry: int = 3, pwd: str = None, ssh_key_file: str = None):
        """
        Execute a command on the VM
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param command Command to execute
        :param logger logger
        :param timeout Timeout in seconds
        :param retry Number of retries
        :param pwd password
        :param ssh_key_file ssh_key file
        :return:
        """
        for i in range(retry):
            try:
                # Construct the SSH client
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                if pwd is not None:
                    # Use password for authentication
                    ssh.connect(mgmt_ip, username=user, password=pwd, timeout=timeout)
                else:
                    pkey = paramiko.RSAKey.from_private_key_file(ssh_key_file)
                    ssh.connect(mgmt_ip, username=user, timeout=timeout, pkey=pkey)

                # Execute the command
                stdin, stdout, stderr = ssh.exec_command(command)
                output = stdout.readlines()
                ssh.close()
                return output
            except Exception as e:
                logger.error(f"Exception : {e}")
                logger.error(traceback.format_exc())
                if i < retry - 1:
                    time.sleep(timeout)
                    logger.info(f"Retrying command {command} on VM {mgmt_ip}")
                else:
                    logger.error(f"Failed to execute command {command} on VM {mgmt_ip}")
                    raise e

    @staticmethod
    def verify_ssh(*, mgmt_ip: str, user: str, logger: logging.Logger, timeout: int = 60, retry: int = 10,
                   pwd: str = None, ssh_key_file: str = None):
        """
        Verify that the VM is accessible via SSH
        :param mgmt_ip Management IP to access the VM
        :param user Default Linux user to use for SSH/Ansible
        :param logger Logger
        :param timeout timeout
        :param retry Number of retries
        :param pwd password
        :param ssh_key_file ssh_key_file

        """
        command = f"echo test ssh from {mgmt_ip} > /tmp/fabric_execute_script.sh; " \
                  f"chmod +x /tmp/fabric_execute_script.sh; /tmp/fabric_execute_script.sh"

        try:
            output = Utils.execute_command(mgmt_ip=mgmt_ip, user=user, command=command, logger=logger,
                                           timeout=timeout, retry=retry, pwd=pwd, ssh_key_file=ssh_key_file)
            logger.info(f"Output: {output}")
        except Exception as e:
            pass

    @staticmethod
    def execute_ansible(*, inventory_path: str, playbook_path: str, extra_vars: dict, logger: logging.Logger,
                        sources: str = None, private_key_file: str = None, host_vars: dict = None,
                        host: str = None, user: str = None):
        """
        Execute ansible
        :param inventory_path: inventory location
        :param playbook_path: playbook
        :param extra_vars: extra vars
        :param logger: logger
        :param sources: sources
        :param private_key_file: private key file
        :param host_vars: host vars
        :param host: host
        :param user: user

        :return OK results
        :raises Exception in case of failure
        """
        ansible_helper = AnsibleHelper(inventory_path=inventory_path, logger=logger, sources=sources)

        ansible_helper.set_extra_vars(extra_vars=extra_vars)

        if host is not None and host_vars is not None and len(host_vars) > 0:
            for key, value in host_vars.items():
                ansible_helper.add_vars(host=host, var_name=key, value=value)

        logger.info(f"Executing playbook {playbook_path} extra_vars: {extra_vars} host_vars: {host_vars}")
        ansible_helper.run_playbook(playbook_path=playbook_path, private_key_file=private_key_file, user=user)
        return ansible_helper.get_result_callback().get_json_result_ok()
