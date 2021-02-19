import argparse
import logging

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.util.id import ID
from fabric_cf.actor.core.core.unit import Unit

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-c", dest='command', required=True, type=str)

    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s",
                        handlers=[logging.StreamHandler()])
    if args.command == 'create':
        u = Unit(uid=ID(uid='u1'))

        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(logger=logger, properties=prop)
        prop2 = {Constants.VM_NAME: "vm1", Constants.WORKER_NODE: "renc-w2.fabric-testbed.net",
                 Constants.HEAD_NODE: "renc-hn.fabric-testbed.net", Constants.FLAVOR: "fabric.large",
                 Constants.IMAGE: "default_centos_8", Constants.PCI_DEVICES: ['[0, 1, 0, 1]']}
        handler.create(unit=u, properties=prop2)
    elif args.command == 'delete':
        u = Unit(uid=ID(uid='u1'))
        prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
        handler = VMHandler(logger=logger, properties=prop)
        prop2 = {Constants.VM_NAME: "vm1", Constants.HEAD_NODE: "renc-hn.fabric-testbed.net"}
        handler.delete(unit=u, properties=prop2)
    else:
        print(f"Unsupported command: {args.command}")
