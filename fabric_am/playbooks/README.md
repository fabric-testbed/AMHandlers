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

#### Attach/Detach FPGA
```bash
ansible-playbook -i inventory fpga_provisioning.yml --extra-vars 'pci_prov_op=attach kvmguest_name=instance-000011d4 worker_node_name=renc-w2.fabric-testbed.net bus=0x25 slot=0x00 fpga_usb_vendor_id=0x0403 fpga_usb_product_id=0x6011'
```

To detach:
```bash
ansible-playbook -i inventory fpga_provisioning.yml --extra-vars 'pci_prov_op=detach kvmguest_name=instance-000011d4 worker_node_name=renc-w2.fabric-testbed.net bus=0x25 slot=0x00 fpga_usb_vendor_id=0x0403 fpga_usb_product_id=0x6011'
```
Note that this rewrites the domain file, does not do hotplug. It attaches however many PCI functions belong to the device
into the VM. To make this change take effect reboot via `openstack server reboot` (note that rebooting via
virsh doesn't work - the changes do not take effect).

Note also that the reboot detaches all hotplugs (and unmounts the volumes).

For testing you can skip `bus`, `slot`, and vendor and product id parameters as sane defaults are present in the role file.
```bash
ansible-playbook -i inventory fpga_provisioning.yml --extra-vars 'pci_prov_op=attach kvmguest_name=instance-000011d4 worker_node_name=renc-w2.fabric-testbed.net'
```
#### Cleanup
Cleanup procedure to be executed for the components. So far, only NVME cleanup support has been added.
##### NVME Cleanup
`nvme_cleanup.yml` ansible script supports cleaning up the NVME drive at the Slice deletion.

Parameters required:
- device: PCI Address of the NVME drive

Example command to cleanup NVME
```
ansible-playbook -i inventory nvme_cleanup.yml --extra-vars 'worker_node_name=renc-w1.fabric-testbed.net device=0000:21:00.0'
```

#### Storage
`head_volume_provisioning.yml` ansible script supports rotating storage operations. 
##### Create a Volume
Create a Volume for a project. This is done out of band of CF manually by the administrator. 

CF requires the volume names to follow the convention:
- Volume Name: `{project-id}-{name}`

Parameters needed:
- Project Id
- Volume Size
- Volume Name
- Life time
- Operation

Example command to create a volume 
```
ansible-playbook head_volume_provisioning.yml -i inventory --extra-vars 'project_id=b9847fa1-13ef-49f9-9e07-ae6ad06cda3f vol_size=10 vol_name=vol1 life_time=10 vol_prov_op=create'
```
Created volume would like as below:
```
[root@renc-hn playbooks(keystone_admin)]# openstack volume show b9847fa1-13ef-49f9-9e07-ae6ad06cda3f-vol1 -f json
{
  "attachments": [],
  "availability_zone": "nova",
  "bootable": "false",
  "consistencygroup_id": null,
  "created_at": "2022-06-20T17:40:34.000000",
  "description": null,
  "encrypted": false,
  "id": "9674170f-5cff-4336-8ada-7a2ce484420c",
  "migration_status": null,
  "multiattach": false,
  "name": "b9847fa1-13ef-49f9-9e07-ae6ad06cda3f-vol1",
  "os-vol-host-attr:host": "renc-hn.fabric-testbed.net@lvm#lvm",
  "os-vol-mig-status-attr:migstat": null,
  "os-vol-mig-status-attr:name_id": null,
  "os-vol-tenant-attr:tenant_id": "3b774ebff5f845898dfcca448d9c658f",
  "properties": {
    "fabric_project_id": "b9847fa1-13ef-49f9-9e07-ae6ad06cda3f",
    "expected_lifetime": "10",
    "fabric_volume_name": "vol1"
  },
  "replication_status": null,
  "size": 10,
  "snapshot_id": null,
  "source_volid": null,
  "status": "available",
  "type": "iscsi",
  "updated_at": "2022-06-20T17:40:36.000000",
  "user_id": "ec1f88832d8a48adb95ace6895b49bef"
}
```
##### Delete a Volume
Delete a Volume for a project. This is done out of band of CF manually by the administrator. 

Parameters needed:
- Project Id
- Volume Name
- Operation
 
Example command to delete a volume
```
ansible-playbook head_volume_provisioning.yml -i inventory --extra-vars 'project_id=b9847fa1-13ef-49f9-9e07-ae6ad06cda3f vol_name=vol1 vol_prov_op=delete'
```
##### Attach a Volume
Attach a Volume to the VM

Parameters needed:
- Project Id
- VM Name
- Volume Name
- Operation

Example command to detach a volume
```
ansible-playbook head_volume_provisioning.yml -i inventory --extra-vars 'project_id=b9847fa1-13ef-49f9-9e07-ae6ad06cda3f vmname=255c7e8b-2ba2-4a99-8f16-66eada8332a8-node-0 vol_name=vol1 vol_prov_op=attach'
```

##### Detach a Volume
Attach a Volume to the VM

Parameters needed:
- Project Id
- VM Name
- Volume Name
- Operation

Example command to detach a volume
```
ansible-playbook head_volume_provisioning.yml -i inventory --extra-vars 'project_id=b9847fa1-13ef-49f9-9e07-ae6ad06cda3f vmname=255c7e8b-2ba2-4a99-8f16-66eada8332a8-node-0 vol_name=vol1 vol_prov_op=detach'
```
