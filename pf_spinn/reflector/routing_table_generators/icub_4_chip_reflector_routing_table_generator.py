from pacman.model.routing_tables.multicast_routing_table import \
    MulticastRoutingTable
from pacman.model.routing_tables.multicast_routing_tables import \
    MulticastRoutingTables
from spinn_machine.multicast_routing_entry import MulticastRoutingEntry
from spinn_machine.utilities.progress_bar import ProgressBar


class ICub4ChipReflectorRoutingTableGenerator(object):
    """ An basic algorithm that can put a hardcoded routing table entry into the
    routers for spinnlink comms
    """

    def __call__(self):

        # progress bar
        progress_bar = ProgressBar(3, "Generating hardcoded routing tables")

        # container
        routing_tables = MulticastRoutingTables()

        # make entries
        multicast_routing_entry_0_0 = \
            MulticastRoutingEntry(0, 0x00000000, [], [3], False)

        multicast_routing_entry_0_1_1_0 = \
            MulticastRoutingEntry(0, 0x00000000, [], [0], False)

        # make routing tables
        chip_0_0_router_table = MulticastRoutingTable(0, 0)
        chip_1_0_router_table = MulticastRoutingTable(1, 0)
        chip_0_1_router_table = MulticastRoutingTable(0, 1)

        # add entry to table
        chip_0_0_router_table.add_multicast_routing_entry(
            multicast_routing_entry_0_0)
        progress_bar.update()
        chip_0_1_router_table.add_multicast_routing_entry(
            multicast_routing_entry_0_1_1_0)
        progress_bar.update()
        chip_1_0_router_table.add_multicast_routing_entry(
            multicast_routing_entry_0_1_1_0)

        # add to container
        routing_tables.add_routing_table(chip_0_0_router_table)
        routing_tables.add_routing_table(chip_1_0_router_table)
        routing_tables.add_routing_table(chip_0_1_router_table)

        progress_bar.end()

        # return tables to stack
        return routing_tables




