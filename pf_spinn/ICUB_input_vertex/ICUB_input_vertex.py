from pacman.model.graphs import AbstractVirtualVertex
from pacman.model.graphs.machine import MachineVertex, \
    MachineSpiNNakerLinkVertex
from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition
from spinn_utilities.overrides import overrides


class ICUBInputVertex(
        MachineVertex, AbstractVirtualVertex, MachineSpiNNakerLinkVertex,
        AbstractProvidesNKeysForPartition):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):

        MachineVertex.__init__(self, label=label, constraints=constraints)
        AbstractVirtualVertex.__init__(self)
        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address)
        AbstractProvidesNKeysForPartition.__init__(self)

    @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    def get_n_keys_for_partition(self, partition, graph_mapper):
        if partition == "Event":
            return 72960
        raise Exception("Incorrect Partition Name at icub input vertex")