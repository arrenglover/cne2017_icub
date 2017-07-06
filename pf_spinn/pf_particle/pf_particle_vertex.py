from pacman.model.decorators.overrides import overrides
from pacman.model.graphs.machine import MachineVertex
from pacman.model.resources import CPUCyclesPerTickResource, DTCMResource
from pacman.model.resources import ResourceContainer, SDRAMResource
from pf_spinn.ICUB_input_vertex.ICUB_input_vertex import ICUBInputVertex
from pf_spinn.pf_agg.pf_agg_vertex import PfAggVertex

from spinn_front_end_common.utilities import constants
from spinn_front_end_common.interface.simulation import simulation_utilities
from spinn_front_end_common.abstract_models.impl.machine_data_specable_vertex \
    import MachineDataSpecableVertex
from spinn_front_end_common.abstract_models.abstract_has_associated_binary \
    import AbstractHasAssociatedBinary
from spinn_front_end_common.utilities.utility_objs.executable_start_type \
    import ExecutableStartType

from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition

from pf_spinn import constants as app_constants

from enum import Enum
import logging

logger = logging.getLogger(__name__)


class PfParticleVertex(
        MachineVertex, MachineDataSpecableVertex, AbstractHasAssociatedBinary,
        AbstractProvidesNKeysForPartition):

    DATA_REGIONS = Enum(
        value="DATA_REGIONS",
        names=[('SYSTEM', 0),
               ('TRANSMISSION_DATA', 1),
               ('RECEPTION_BASE_KEYS', 2),
               ('CONFIG', 3)])

    CORE_APP_IDENTIFIER = 0xBEEF
    TRANSMISSION_DATA_SIZE = 16
    RECEPTION_KEY_SIZE = 8
    CONFIG_PARAM_SIZE = 16

    def __init__(self, x, y, r, packet_threshold, label, id, constraints=None):
        MachineVertex.__init__(self, label=label, constraints=constraints)
        
        AbstractProvidesNKeysForPartition.__init__(self)

        self._x = x
        self._y = y
        self._r = r
        self._packet_threshold = packet_threshold
        self._placement = None
        self._id = id

    @property
    @overrides(MachineVertex.resources_required)
    def resources_required(self):
        sdram_required = (
            constants.SYSTEM_BYTES_REQUIREMENT +
            self.TRANSMISSION_DATA_SIZE + self.RECEPTION_KEY_SIZE +
            self.CONFIG_PARAM_SIZE)
        resources = ResourceContainer(
            cpu_cycles=CPUCyclesPerTickResource(45),
            dtcm=DTCMResource(100), sdram=SDRAMResource(sdram_required))
        return resources

    @overrides(AbstractHasAssociatedBinary.get_binary_file_name)
    def get_binary_file_name(self):
        return "pf_particle.aplx"

    @overrides(AbstractHasAssociatedBinary.get_binary_start_type)
    def get_binary_start_type(self):
        return ExecutableStartType.USES_SIMULATION_INTERFACE
        
    @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    def get_n_keys_for_partition(self, partition, graph_mapper):
        return 5

    @overrides(MachineDataSpecableVertex.generate_machine_data_specification)
    def generate_machine_data_specification(
            self, spec, placement, machine_graph, routing_info, iptags,
            reverse_iptags, machine_time_step, time_scale_factor):

        self._placement = placement

        # Setup words + 1 for flags + 1 for recording size
        setup_size = constants.SYSTEM_BYTES_REQUIREMENT

        # Create the data regions for hello world
        self._reserve_memory_regions(spec, setup_size)

        # write data for the simulation data item
        spec.switch_write_focus(self.DATA_REGIONS.SYSTEM.value)
        spec.write_array(simulation_utilities.get_simulation_header_array(
            self.get_binary_file_name(), machine_time_step,
            time_scale_factor))

        # write transmission key
        spec.switch_write_focus(self.DATA_REGIONS.TRANSMISSION_DATA.value)
        routing_key = routing_info.get_first_key_from_partition(
            "Particle State")
        if routing_key is None:
            spec.write_value(0)
            spec.write_value(0)
            spec.write_value(0)
        else:
            spec.write_value(1)
            spec.write_value(routing_key)
            spec.write_value(self._id)

        # write reception key
        spec.switch_write_focus(self.DATA_REGIONS.RECEPTION_BASE_KEYS.value)
        input_vertex = None
        agg_vertex = None
        for edge in machine_graph.get_edges_ending_at_vertex(self):
            if isinstance(edge.pre_vertex, ICUBInputVertex):
                input_vertex = edge.pre_vertex
            if isinstance(edge.pre_vertex, PfAggVertex):
                agg_vertex = edge.pre_vertex

        # retina key
        routing_key = routing_info.get_first_key_from_pre_vertex(
            input_vertex, app_constants.EDGE_PARTITION_EVENT)
        if routing_key is None:
            raise Exception("The particle is not receiving from the retina")
        spec.write_value(routing_key)

        # agg key
        routing_key = routing_info.get_first_key_from_pre_vertex(
            agg_vertex, app_constants.EDGE_PARTITION_RE_SAMPLE)
        if routing_key is None:
            raise Exception(
                "The particle is not receiving from the aggregation core")
        spec.write_value(routing_key)

        # write config params
        spec.switch_write_focus(self.DATA_REGIONS.CONFIG.value)
        spec.write_value(self._x)
        spec.write_value(self._y)
        spec.write_value(self._r)
        spec.write_value(self._packet_threshold)

        # End-of-Spec:
        spec.end_specification()

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
            label="Particle Keys")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.CONFIG.value,
            size=self.CONFIG_PARAM_SIZE,
            label="Config x, y, r")
