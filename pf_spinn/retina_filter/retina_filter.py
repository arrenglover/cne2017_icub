from pacman.model.graphs.machine import MachineVertex
from pacman.model.resources import ResourceContainer, \
    CPUCyclesPerTickResource, DTCMResource, SDRAMResource
from spinn_front_end_common.abstract_models import \
    AbstractHasAssociatedBinary, \
    AbstractProvidesNKeysForPartition
from spinn_front_end_common.abstract_models.impl import \
    MachineDataSpecableVertex
from spinn_front_end_common.interface.simulation import simulation_utilities
from spinn_front_end_common.utilities import constants
from spinn_front_end_common.utilities.utility_objs import ExecutableType
from pf_spinn import constants as app_constants

class RetinaFilter(
        MachineVertex, MachineDataSpecableVertex, AbstractHasAssociatedBinary,
        AbstractProvidesNKeysForPartition):

    CORE_APP_IDENTIFIER = 0xBEEF
    TRANSMISSION_DATA_SIZE = 16
    RECEPTION_KEY_SIZE = 8


    def __init__(
            self, partition_identifier, filter, label=None,
            constraints=None):
        MachineVertex.__init__(self, label, constraints)
        MachineDataSpecableVertex.__init__(self)
        AbstractHasAssociatedBinary.__init__(self)
        AbstractProvidesNKeysForPartition.__init__(self)
        self._partition_identifier = partition_identifier
        self._filter = filter

    @property
    def resources_required(self):
        sdram_required = (
            constants.SYSTEM_BYTES_REQUIREMENT +
            self.TRANSMISSION_DATA_SIZE + self.RECEPTION_KEY_SIZE)
        resources = ResourceContainer(
            cpu_cycles=CPUCyclesPerTickResource(45),
            dtcm=DTCMResource(100), sdram=SDRAMResource(sdram_required))
        return resources

    def get_n_keys_for_partition(self, partition, graph_mapper):
        if partition == self._partition_identifier:
            return app_constants.RETINA_Y_SIZE
        else:
            raise Exception("SOMETHING GONE BOOMY! GOT THE WRONG PARTITION "
                            "IDENTIFIER")

    def generate_machine_data_specification(
            self, spec, placement, machine_graph, routing_info,
            iptags, reverse_iptags, machine_time_step, time_scale_factor):
        # Create the data regions for hello world
        self._reserve_memory_regions(spec, constants.SYSTEM_BYTES_REQUIREMENT)
        # write data for the simulation data item
        spec.switch_write_focus(self.DATA_REGIONS.SYSTEM.value)
        spec.write_array(simulation_utilities.get_simulation_header_array(
            self.get_binary_file_name(), machine_time_step,
            time_scale_factor))

        # write transmission key
        spec.switch_write_focus(self.DATA_REGIONS.TRANSMISSION_DATA.value)

        routing_key = routing_info.get_first_key_from_pre_vertex(
            self, self._partition_identifier)

        if routing_key is None:
            raise Exception("FUCK")
        if routing_key is None:
            spec.write_value(0)
            spec.write_value(0)
            spec.write_value(0)
        else:
            spec.write_value(1)
            spec.write_value(routing_key)
            spec.write_value(self._id)

    def _reserve_memory_regions(self, spec, system_size):
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.SYSTEM.value, size=system_size,
            label='systemInfo')
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.TRANSMISSION_DATA.value,
            size=self.TRANSMISSION_DATA_SIZE,
            label="My Key")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.RECEPTION_BASE_KEYS.value,
            size=self.RECEPTION_KEY_SIZE,
            label="filter")

    def get_binary_file_name(self):
        return "retina_filter.aplx"

    def get_binary_start_type(self):
        return ExecutableType.USES_SIMULATION_INTERFACE
