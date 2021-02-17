# Playbooks

Playbooks used to interface with openstack

## Requirements

1. Populate inventory file, `inventory`. Initial version looks like this
(with bogus entries):
```
  # file: fabric

[fabric_site1_worker]
fabric-site1-w1 ansible_host=192.168.101.101
fabric-site1-w2 ansible_host=192.168.101.102
fabric-site1-w3 ansible_host=192.168.101.103

[fabric_site1_head]
fabric-site1-head ansible_host=192.168.101.100

[fabric_site2_head]
fabric-site2-head ansible_host=192.168.102.100

[fabric_worker:children]
fabric_site1_worker

[fabric_head:children]
fabric_site1_head
fabric_site2_head
```

2. Create an `ansible.cfg` file as needed. 

3. Name of the head node and the openstack instance we want to find, and the
PCI address of the card we want to add/remove from the instance.

## Operation
There are currently two playbooks:

### `head.yml`

Finds (and returns the name of) which worker node
is running the openstack KVM-based (i.e. virtual) instance we are interested on.

### `worker.yml`

### Using playbooks

1. Ask openstack running in `fabric_site1_head` which worker node has the instance `testinstance`:

```bash
ansible-playbook -i inventory head.yml --extra-vars 'headnode_name=fabric-site1-head instance_name=testinstance'
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

2. Add the PCI card, with address 0000:01:0.1, to  `kvmguest_name`. Note that we have to make `kvmguest_name` match its name in the inventory.

```bash
ansible-playbook -i inventory worker.yml --extra-vars 'kvmguest_name=instance-00000040 workernode_name=fabric-site1-w3 add_pcidevice=True' --extra-vars '{"pcidevice_address": "[0, 1, 0, 1]"}'
```
Note the number of zeros on the left of each number in `pcidevice_address` are not important.

Returns (for successful run):
```bash
                            "stderr_lines": [],
                            "stdout": "Device attached successfully",
                            "stdout_lines": [
                                "Device attached successfully"
                            ]
```
3. Remove the PCI card

```bash
ansible-playbook -i inventory worker.yml --extra-vars 'kvmguest_name=instance-00000040 workernode_name=fabric-site1-w3 add_pcidevice=False' --extra-vars '{"pcidevice_address": "[0,1,0,1]"}'
```

Note the space between the numbers in the `pcidevice_address` does not matter.

Returns (for successful run):
```bash
                            "stderr_lines": [],
                            "stdout": "Device detached successfully",
                            "stdout_lines": [
                                "Device detached successfully"
                            ]
```

- 
