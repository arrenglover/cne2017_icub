from pacman.model.constraints.key_allocator_constraints import \
    KeyAllocatorFixedKeyAndMaskConstraint
from pacman.model.graphs.machine import MachineSpiNNakerLinkVertex
from pacman.model.routing_info import BaseKeyAndMask
from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition
from spinn_front_end_common.abstract_models.\
    abstract_provides_outgoing_partition_constraints import \
    AbstractProvidesOutgoingPartitionConstraints
from spinn_utilities.overrides import overrides
from pf_spinn import constants


class ICUBOutputVertex(
        MachineSpiNNakerLinkVertex, AbstractProvidesNKeysForPartition,
        AbstractProvidesOutgoingPartitionConstraints):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):

        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address, label=label, constraints=constraints)
        AbstractProvidesNKeysForPartition.__init__(self)
        AbstractProvidesOutgoingPartitionConstraints.__init__(self)

    @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    def get_n_keys_for_partition(self, partition, graph_mapper):
        if partition.identifier == constants.EDGE_PARTITION_EVENT:
            return 1048576
        raise Exception("Incorrect Partition Name at icub output vertex")

    @overrides(AbstractProvidesOutgoingPartitionConstraints.
               get_outgoing_partition_constraints)
    def get_outgoing_partition_constraints(self, partition):
        return [KeyAllocatorFixedKeyAndMaskConstraint(
            keys_and_masks=[BaseKeyAndMask(
                base_key=0x00200000, mask=0xFFF00000)])]

