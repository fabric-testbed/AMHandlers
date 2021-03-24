# Playbooks

Playbooks used to interface with openstack

## Requirements

1. Populate inventory file, `inventory`. Initial version looks like this
(with bogus entries):
```
  # file: fabric

[fabric_site_worker]
fabric-site1-w1 ansible_host=192.168.101.101
fabric-site1-w2 ansible_host=192.168.101.102
fabric-site1-w3 ansible_host=192.168.101.103

[fabric_site_head]
fabric-site1-head ansible_host=192.168.101.100
fabric-site1-head auth_url=
fabric-site1-head password=
fabric-site1-head project_name=admin
fabric-site1-head username=admin
fabric-site1-head net_name=management-2004
fabric-site1-head security_group=fully-permissive
fabric-site1-head key_name=default
fabric-site1-head ext_network=public

[fabric_worker:children]
fabric_site1_worker

[fabric_head:children]
fabric_site1_head
```

2. Create an `ansible.cfg` file as needed. 

## Operation

### `head_vm_provisioning.yml`
Supports following operations:
- Provision a VM
  - Parameters:  vm_prov_op: 'create', vm_name, image, flavor, availability_zone
- Attach Floating IP Address to a VM
  - Parameters: vm_prov_op: 'attach_fip', vm_name
- Delete a VM
  - Parameters: vm_prov_op: 'delete', vm_name
- Get a VM
  - Parameters: vm_prov_op: 'get', vm_name
### `head_get_worker_node_for_vm.yml`
Finds (and returns the name of) which worker node
is running the openstack KVM-based (i.e. virtual) instance we are interested on.

### `worker_pci_provisioning.yml`
Supports following operations:
- Attach a PCI Device to a VM
  - Parameters:  pci_prov_op: 'attach', device, kvmguest_name, worker_node_name, domain, bus, slot, function
- Detach a PCI Device from a VM
  - Parameters:  pci_prov_op: 'detach', device, kvmguest_name, worker_node_name, domain, bus, slot, function
### Using playbooks
#### VM Provisioning
- Create a VM instance
```bash
ansible-playbook -i inventory head_vm_provisioning.yml --extra-vars 'vm_prov_op=create vm_name=vm1 image=default_centos_8 flavor=fabric.large availability_zone=nova:renc-w1.fabric-testbed.net'
```
- Attach a Floating IP
```bash
ansible-playbook -i inventory head_vm_provisioning.yml --extra-vars 'vm_prov_op=attach_fip vm_name=vm1'
```
- Delete a VM instance
```bash
ansible-playbook -i inventory head_vm_provisioning.yml --extra-vars 'vm_prov_op=delete vm_name=vm1'
```
- Get a VM instance
```bash
ansible-playbook -i inventory head_vm_provisioning.yml --extra-vars 'vm_prov_op=get vm_name=vm1'
```

#### Ask openstack running in `fabric_site1_head` which worker node has the instance `testinstance`:

```bash
ansible-playbook -i inventory head_get_worker_node_for_vm.yml --extra-vars 'headnode_name=fabric-site1-head instance_name=testinstance'
```

Returns:
```bash
                                "kvmguest_name": "instance-00000040",
                                "kvmguest_status": "ACTIVE",
                                "workernode_name": "fabric-site1-w3.fabric-testbed.net"
```
where
- `kvmguest_name`: Name used by KVM for the openstack instance we are looking for.
- `kvmguest_status`: Current status of the instance. In this case it is running (`ACTIVE`)
- `workernode_name`: The name of the worker node running the instance. 

#### Add the PCI card, with address 0000:01:0.1, to  `kvmguest_name`. Note that we have to make `kvmguest_name` match its name in the inventory.
```bash
ansible-playbook -i inventory worker_pci_provisioning.yml --extra-vars 'pci_prov_op=attach device=device1 \
kvmguest_name=instance-00000040 worker_node_name=fabric-site1-w3 domain=0x00 bus=0x1 slot=0x0 function=0x01'
```

Returns (for successful run):
```bash
                            "stderr_lines": [],
                            "stdout": "Device attached successfully",
                            "stdout_lines": [
                                "Device attached successfully"
                            ]
```
#### Remove the PCI card

```bash
ansible-playbook -i inventory worker_pci_provisioning.yml --extra-vars 'pci_prov_op=detach device=device1 \
kvmguest_name=instance-00000040 worker_node_name=fabric-site1-w3 domain=0x00 bus=0x1 slot=0x0 function=0x01'
```

Returns (for successful run):
```bash
                            "stderr_lines": [],
                            "stdout": "Device detached successfully",
                            "stdout_lines": [
                                "Device detached successfully"
                            ]
```