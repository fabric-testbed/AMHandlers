from fabric_cf.actor.core.common.constants import Constants

from fabric_am.handlers.vm_handler import VMHandler
from fabric_am.util.am_constants import AmConstants

if __name__ == '__main__':
    import logging

    from fabric_cf.actor.core.util.id import ID
    from fabric_cf.actor.core.core.unit import Unit
    u = Unit(uid=ID(uid='u1'))
    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s",
                        handlers=[logging.StreamHandler()])

    prop = {AmConstants.CONFIG_PROPERTIES_FILE: 'fabric_am/config/vm_handler_config.yml'}
    handler = VMHandler(logger=logger, properties=prop)
    prop2 = {Constants.VM_NAME: "vm1", Constants.WORKER_NODE: "uky-w2.fabric-testbed.net",
             Constants.HEAD_NODE: "uky-hn.fabric-testbed.net", Constants.FLAVOR: "fabric.large"}
    handler.create(unit=u, properties=prop2)