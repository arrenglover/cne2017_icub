from pacman.model.graphs import AbstractVirtualVertex
from pacman.model.graphs.machine import MachineVertex, \
    MachineSpiNNakerLinkVertex


class ICUBOutputVertex(AbstractVirtualVertex, MachineVertex,
                       MachineSpiNNakerLinkVertex):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):
        AbstractVirtualVertex.__init__(self)
        MachineVertex.__init__(self, label=label, constraints=constraints)
        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address)

