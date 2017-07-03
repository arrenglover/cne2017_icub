from pacman.model.graphs.machine import MachineSpiNNakerLinkVertex
from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition
from spinn_utilities.overrides import overrides
from pf_spinn import constants


class ICUBInputVertex(
        MachineSpiNNakerLinkVertex, AbstractProvidesNKeysForPartition):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):

        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address, label=label, constraints=constraints)
        AbstractProvidesNKeysForPartition.__init__(self)

    @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    def get_n_keys_for_partition(self, partition, graph_mapper):
        if partition.identifier == constants.EDGE_PARTITION_EVENT:
            return 72960
        raise Exception("Incorrect Partition Name at icub input vertex")