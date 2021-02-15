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
# Create a callback plugin so we can capture the output
import json
import os
import traceback
from typing import List

from ansible import context
from ansible.executor.playbook_executor import PlaybookExecutor
from ansible.inventory.manager import InventoryManager
from ansible.module_utils.common.collections import ImmutableDict
from ansible.parsing.dataloader import DataLoader
from ansible.plugins.callback import CallbackBase
from ansible.vars.manager import VariableManager


class ResultsCollectorJSONCallback(CallbackBase):
    """A callback plugin used for performing an action as results come in.

    If you want to collect all results into a single object for processing at
    the end of the execution, look into utilizing the ``json`` callback plugin
    or writing your own custom callback plugin.
    """

    def __init__(self, *args, **kwargs):
        super(ResultsCollectorJSONCallback, self).__init__(*args, **kwargs)
        self.host_ok = {}
        self.host_unreachable = {}
        self.host_failed = {}

    def v2_runner_on_unreachable(self, result):
        """
        Store the result in an instance attribute for retrieval later
        @param result result
        """
        host = result._host
        self.host_unreachable[host.get_name()] = result

    def v2_runner_on_ok(self, result, *args, **kwargs):
        """
        Store the result in an instance attribute for retrieval later
        @param result result
        """
        host = result._host
        self.host_ok[host.get_name()] = result

    def v2_runner_on_failed(self, result, *args, **kwargs):
        """
        Store the result in an instance attribute for retrieval later
        @param result result
        """
        host = result._host
        self.host_failed[host.get_name()] = result

    def get_json_result(self, host: str, host_result_map: dict):
        """
        Get Json Result for a host
        @param host host
        @param host_result_map map containing host results
        """
        result = host_result_map.get(host, None)
        if result is not None:
            return json.dumps(result._result)

    def get_json_result_ok(self, host: str):
        """
        Get Json OK Result for a host
        @param host host
        """
        return self.get_json_result(host=host, host_result_map=self.host_ok)

    def get_json_result_unreachable(self, host: str):
        """
        Get Json Unreachable Result for a host
        @param host host
        """
        return self.get_json_result(host=host, host_result_map=self.host_unreachable)

    def get_json_result_failed(self, host: str):
        """
        Get Json Failed Result for a host
        @param host host
        """
        return self.get_json_result(host=host, host_result_map=self.host_failed)


class AnsibleHelper:
    """
    Helper class to invoke the Ansible Playbook
    """
    def __init__(self, hosts: List[str], logger):
        self.results_callback = ResultsCollectorJSONCallback()
        self.loader = DataLoader()
        self.inventory = InventoryManager(loader=self.loader, sources=hosts)
        self.variable_manager = VariableManager(loader=self.loader, inventory=self.inventory)
        self.logger = logger

    def add_vars(self, host: str, var_name: str, value: str):
        """
        Set environment variables needed by the playbook
        @param host host
        @param var_name variable name
        @param value value
        """
        self.variable_manager.set_host_variable(host=host, varname=var_name, value=value)

    def run_playbook(self, playbook_path: str) -> bool:
        """
        Run a playbook
        @param playbook_path path for the playbook
        @return True if playbook ran and false otherwise
        """
        if not os.path.exists(playbook_path):
            raise Exception("Playbook not found")

        context.CLIARGS = ImmutableDict(connection='smart', tags={}, listtags=False, listtasks=False, listhosts=False,
                                        syntax=False,
                                        module_path=None, forks=100, private_key_file=None,
                                        ssh_common_args=None, ssh_extra_args=None, sftp_extra_args=None,
                                        scp_extra_args=None, become=False,
                                        become_method='sudo', become_user='root', verbosity=True, check=False,
                                        start_at_task=None)
        passwords = {}

        pbex = PlaybookExecutor(playbooks=[playbook_path], inventory=self.inventory,
                                variable_manager=self.variable_manager,
                                loader=self.loader, passwords=passwords)

        pbex._tqm._stdout_callback = self.results_callback
        try:
            results = pbex.run()
            return True
        except Exception as e:
            self.logger.error(f"Exception occurred while executing playbook {playbook_path} e: {e}")
            self.logger.error(traceback.format_exc())
        finally:
            if self.loader is not None:
                self.loader.cleanup_all_tmp_files()
        return False

    def get_result_callback(self):
        """
        Fetch the Result Callback which contains all the results
        @return result callback
        """
        return self.results_callback
