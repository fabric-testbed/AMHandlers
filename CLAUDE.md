# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**fabric_am_handlers** — Resource handlers for FABRIC testbed Aggregate Managers (AMs). Each handler manages substrate-specific provisioning (VMs, networks, switches, storage, PCI devices) by executing Ansible playbooks and SSH commands against site infrastructure.

Part of the [FABRIC Control Framework](https://github.com/fabric-testbed/ControlFramework). Version tracked in `fabric_am/__init__.py`.

## Build & Install

```bash
# Uses flit as build backend
pip install flit
flit install            # install in development mode
flit build              # build distribution
```

## Testing

```bash
# Unit tests (pytest)
pip install pytest
pytest fabric_am/test/

# Run a single test file
pytest fabric_am/test/test_vm_handler.py

# Integration playbook tests (requires infrastructure access)
python test_playbooks.py
```

Test files use `handler.test_mode = True` to bypass actual infrastructure calls.

## Architecture

### Handler Pattern

All handlers inherit from `HandlerBase` (from `fabric_cf`) and implement:
- `create()` — provision a resource
- `delete()` — deprovision a resource
- `modify()` — update resource state (attach/detach devices, reboot, etc.)
- `clean_restart()` — full cleanup for resource reset

Each handler has a corresponding YAML config in `fabric_am/config/` specifying playbook paths, runtime settings, and image mappings.

### Handlers

| Handler | File | Purpose |
|---------|------|---------|
| `VMHandler` | `handlers/vm_handler.py` | VM lifecycle, PCI passthrough (GPU/SmartNIC/FPGA/NVME), floating IPs, CPU pinning, NUMA tuning |
| `NetHandler` | `handlers/net_handler.py` | L2/L3 network services via NSO REST API |
| `Al2sHandler` | `handlers/al2s_handler.py` | AL2S (Advanced Layer 2 Services) network provisioning |
| `OessHandler` | `handlers/oess_handler.py` | OESS (OpenFlow Enabled Site Switch) operations |
| `SwitchHandler` | `handlers/switch_handler.py` | Physical/logical switch configuration via Ansible |
| `VnicNetHandler` | `handlers/vnic_net_handler.py` | OpenStack vNIC creation on site head nodes |

### Key Utilities

- **`util/ansible_helper.py`** — `AnsibleHelper` wraps Ansible Runner with custom `ResultsCollectorJSONCallback` for capturing playbook results
- **`util/utils.py`** — SSH execution with retry logic (paramiko), CPU/NUMA info parsing, playbook execution wrappers
- **`util/am_constants.py`** — All magic strings and operation type constants

### Provisioning Flow

Handlers receive resource configuration via `ConfigToken` from the Control Framework, translate it into Ansible playbook variables, and execute playbooks from `fabric_am/playbooks/`. Playbooks use roles in `playbooks/roles/` for modular provisioning steps.

## Dependencies

- `ansible==13.6.0` — playbook execution (pinned version; bundles ansible-core ~2.20)
- `paramiko` — SSH remote execution
- `fabric-cf==1.9.1` — Control Framework base classes and FIM (Fabric Information Model)
- Python >= 3.12

## Key Conventions

- Custom exception per handler (e.g., `VmHandlerException`)
- Handler results returned as dictionaries with status codes
- YAML configs pair each handler with its playbooks and runtime settings
- Image-to-default-user mappings defined in handler configs (e.g., `default_ubuntu_22: ubuntu`)
