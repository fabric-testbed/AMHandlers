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
import ipaddress
import uuid
from collections import OrderedDict
from enum import Enum, IntEnum, StrEnum

from fabric_am.util.ansible_helper import to_ansible_native

try:
    # private ansible internals: only used to prove taggability, so tolerate
    # them moving in a future ansible-core release
    from ansible._internal._datatag import _tags
    from ansible.module_utils._internal._datatag import AnsibleTagHelper
except ImportError:  # pragma: no cover
    _tags = None
    AnsibleTagHelper = None


class SampleType(Enum):
    VM = "vm"


class SampleStrType(StrEnum):
    NET = "net"


class SampleIntType(IntEnum):
    THREE = 3


def test_native_types_are_unchanged():
    values = {"user": "rocky", "count": 3, "ratio": 1.5, "flag": True, "empty": None}
    assert to_ansible_native(values) == values


def test_ip_addresses_are_stringified():
    ipv6 = ipaddress.IPv6Address("2001:1948:417:7:f816:3eff:fe17:ca17")
    ipv4 = ipaddress.IPv4Address("10.0.0.1")

    assert to_ansible_native(ipv6) == str(ipv6)
    assert to_ansible_native({"vmname": ipv4}) == {"vmname": "10.0.0.1"}


def test_nested_structures_are_coerced():
    extra_vars = {
        "vmname": ipaddress.IPv6Address("2001:1948:417:7:f816:3eff:fe17:ca17"),
        "user": "rocky",
        "keys": [{"key": "ssh-rsa AAAA", "comment": "addkey-by-poa-fablib"}],
        "operation": "addkey",
        "poa_id": uuid.UUID("12345678123456781234567812345678"),
        "type": SampleType.VM,
        "hosts": (ipaddress.IPv4Address("10.0.0.1"), 2),
    }

    result = to_ansible_native(extra_vars)

    assert result["vmname"] == "2001:1948:417:7:f816:3eff:fe17:ca17"
    assert result["keys"] == [{"key": "ssh-rsa AAAA", "comment": "addkey-by-poa-fablib"}]
    assert result["poa_id"] == "12345678-1234-5678-1234-567812345678"
    assert result["type"] == "vm"
    assert result["hosts"] == ["10.0.0.1", 2]

    assert_taggable(result)


def test_scalar_subclasses_are_rebuilt_as_exact_types():
    """
    ansible tags by exact type, so subclasses of the native scalars are not
    taggable even though isinstance() says they are.
    """
    class MyStr(str):
        pass

    class MyInt(int):
        pass

    class MyFloat(float):
        pass

    result = to_ansible_native({
        "str_enum": SampleStrType.NET,
        "int_enum": SampleIntType.THREE,
        "str_sub": MyStr("x"),
        "int_sub": MyInt(7),
        "float_sub": MyFloat(1.5),
        "ordered": OrderedDict([("k", ipaddress.IPv4Address("10.0.0.1"))]),
    })

    assert result["str_enum"] == "net" and type(result["str_enum"]) is str
    assert result["int_enum"] == 3 and type(result["int_enum"]) is int
    assert result["str_sub"] == "x" and type(result["str_sub"]) is str
    assert result["int_sub"] == 7 and type(result["int_sub"]) is int
    assert result["float_sub"] == 1.5 and type(result["float_sub"]) is float
    assert result["ordered"] == {"k": "10.0.0.1"} and type(result["ordered"]) is dict

    assert_taggable(result)


def assert_taggable(value, path: str = "root"):
    """
    Assert ansible can tag every value in the coerced structure, i.e. the
    playbook load that raised "is not taggable" would now succeed.
    """
    if isinstance(value, dict):
        assert type(value) is dict, f"{path}: {type(value)}"
        for k, v in value.items():
            assert type(k) is str, f"{path} key {k!r}: {type(k)}"
            assert_taggable(v, f"{path}.{k}")
        return
    if isinstance(value, list):
        assert type(value) is list, f"{path}: {type(value)}"
        for i, v in enumerate(value):
            assert_taggable(v, f"{path}[{i}]")
        return
    if value is None:
        return
    assert type(value) in (str, bool, int, float), f"{path}: {type(value)}"
    if AnsibleTagHelper is not None:
        # raises NotTaggableError for anything ansible would reject
        AnsibleTagHelper.tag(value, _tags.Origin(description="test"))
