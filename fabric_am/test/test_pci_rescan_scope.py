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
import threading
import unittest

from fim.slivers.attached_components import ComponentSliver, ComponentType, AttachedComponentsInfo
from fim.slivers.capacities_labels import Labels
from fim.slivers.network_node import NodeSliver

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants


class TestPciRescanScope(unittest.TestCase):
    """
    A POA rescan resets a PCI function. Expanding that reset to the sibling functions of the
    same card is correct for an FPGA and destructive for a NIC whose sibling PF carries the
    SR-IOV VFs of other slivers.
    """
    logger = logging.getLogger(__name__)
    prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'config/vm_handler_config.yml'}
    handler = VMHandler(logger=logger, properties=prop, process_lock=threading.Lock())
    handler.test_mode = True

    # name-mangled private method
    allowed = staticmethod(lambda sliver, bdf:
                           TestPciRescanScope.handler._VMHandler__rescan_siblings_allowed(sliver=sliver, bdf=bdf))

    @staticmethod
    def _sliver(*components) -> NodeSliver:
        sliver = NodeSliver()
        sliver.set_name('vm-1')
        aci = AttachedComponentsInfo()
        for name, ctype, model, bdf in components:
            c = ComponentSliver()
            c.set_name(name)
            c.set_type(ctype)
            c.set_model(model)
            c.set_labels(Labels(bdf=bdf))
            aci.add_device(c)
        sliver.attached_components_info = aci
        return sliver

    def test_fpga_may_expand_to_siblings(self):
        """An FPGA records one BDF but the whole card belongs to the sliver."""
        sliver = self._sliver(('fpga1', ComponentType.FPGA, 'Xilinx-U280', ['0000:25:00.0']))
        self.assertTrue(self.allowed(sliver, ['0000:25:00.0']))

    def test_dedicated_nic_never_expands(self):
        """The sibling PF of a mixed-mode card serves other slivers' VFs."""
        sliver = self._sliver(('nic1', ComponentType.SmartNIC, 'ConnectX-6-1P', ['0000:41:00.1']))
        self.assertFalse(self.allowed(sliver, ['0000:41:00.1']))

    def test_shared_nic_never_expands(self):
        sliver = self._sliver(('nic1', ComponentType.SharedNIC, 'ConnectX-6', ['0000:41:00.2']))
        self.assertFalse(self.allowed(sliver, ['0000:41:00.2']))

    def test_mixed_match_is_refused(self):
        """If the request touches an FPGA and a NIC, the unsafe one decides."""
        sliver = self._sliver(('fpga1', ComponentType.FPGA, 'Xilinx-U280', ['0000:25:00.0']),
                              ('nic1', ComponentType.SmartNIC, 'ConnectX-6-1P', ['0000:41:00.1']))
        self.assertFalse(self.allowed(sliver, ['0000:25:00.0', '0000:41:00.1']))
        # the FPGA on its own is still fine
        self.assertTrue(self.allowed(sliver, ['0000:25:00.0']))

    def test_unmatched_or_empty_is_refused(self):
        sliver = self._sliver(('fpga1', ComponentType.FPGA, 'Xilinx-U280', ['0000:25:00.0']))
        self.assertFalse(self.allowed(sliver, ['0000:99:00.0']))
        self.assertFalse(self.allowed(sliver, []))
        self.assertFalse(self.allowed(sliver, None))

    def test_no_components_is_refused(self):
        sliver = NodeSliver()
        sliver.set_name('vm-1')
        self.assertFalse(self.allowed(sliver, ['0000:25:00.0']))

    def test_scalar_bdf_label_is_handled(self):
        """labels.bdf is a string on some components rather than a list."""
        sliver = self._sliver(('fpga1', ComponentType.FPGA, 'Xilinx-U280', '0000:25:00.0'))
        self.assertTrue(self.allowed(sliver, ['0000:25:00.0']))


if __name__ == '__main__':
    unittest.main()
