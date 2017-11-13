import struct

from pacman.executor.injection_decorator import inject_items
from pacman.model.decorators.overrides import overrides
from pacman.model.graphs.machine import MachineVertex
from pacman.model.resources import CPUCyclesPerTickResource, DTCMResource
from pacman.model.resources import ResourceContainer, SDRAMResource

from spinn_front_end_common.utilities import helpful_functions
from spinn_front_end_common.interface.simulation import simulation_utilities
from spinn_front_end_common.abstract_models.impl.machine_data_specable_vertex \
    import MachineDataSpecableVertex
from spinn_front_end_common.abstract_models.abstract_has_associated_binary \
    import AbstractHasAssociatedBinary
from spinn_front_end_common.interface.buffer_management.buffer_models\
    .abstract_receive_buffers_to_host import AbstractReceiveBuffersToHost
from spinn_front_end_common.interface.buffer_management\
    import recording_utilities
from spinn_front_end_common.utilities.utility_objs.executable_type \
    import ExecutableStartType

from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition

from spinn_front_end_common.utilities import globals_variables

from spinn_front_end_common.utilities import constants

from enum import Enum
import logging

from pf_spinn import constants as app_constants

logger = logging.getLogger(__name__)


class PfAggVertex(
        MachineVertex, MachineDataSpecableVertex, AbstractHasAssociatedBinary,
        AbstractReceiveBuffersToHost, AbstractProvidesNKeysForPartition):

    DATA_REGIONS = Enum(
        value="DATA_REGIONS",
        names=[('SYSTEM', 0),
               ('TRANSMISSION_DATA', 1),
               ('RECEPTION_BASE_KEYS', 2),
               ('RECORDED_DATA', 3),
               ('CONFIG', 4)])

    CORE_APP_IDENTIFIER = 0xBEEF

    KEYS_PER_TARGET_POSITION = 1048576
    KEYS_PER_SAMPLE_DATA = 6

    SDRAM_PER_TIMER_TICK_PER_RECORDING = 12
    TRANSMISSION_DATA_SIZE = 16
    CONFIG_DATA_REGION_SIZE = 8

    RECORDED_REGION_ID = 0

    def __init__(self, label, n_particles, constraints=None,
                 record_data=False):
        MachineVertex.__init__(self, label=label, constraints=constraints)
        
        AbstractProvidesNKeysForPartition.__init__(self)
        
        self._config = globals_variables.get_simulator().config

        self._buffer_size_before_receive = None
        if self._config.getboolean("Buffers", "enable_buffered_recording"):
            self._buffer_size_before_receive = self._config.getint(
                "Buffers", "buffer_size_before_receive")
        self._time_between_requests = self._config.getint(
            "Buffers", "time_between_requests")
        self._receive_buffer_host = self._config.get(
            "Buffers", "receive_buffer_host")
        self._receive_buffer_port = helpful_functions.read_config_int(
            self._config, "Buffers", "receive_buffer_port")
        self.n_particles = n_particles
        self._record_data = record_data
        self._placement = None

    @property
    @inject_items({"n_machine_time_steps": "TotalMachineTimeSteps"})
    @overrides(MachineVertex.resources_required,
               additional_arguments=["n_machine_time_steps"])
    def resources_required(self, n_machine_time_steps):
        sdram_required = (
            constants.SYSTEM_BYTES_REQUIREMENT +
            self.TRANSMISSION_DATA_SIZE + 4 + (self.n_particles * 4) + 4 +
            self.CONFIG_DATA_REGION_SIZE)

        if self._record_data:
            sdram_required += (
                n_machine_time_steps *
                self.SDRAM_PER_TIMER_TICK_PER_RECORDING)
        
        resources = ResourceContainer(
            cpu_cycles=CPUCyclesPerTickResource(45),
            dtcm=DTCMResource(100), sdram=SDRAMResource(sdram_required))

        resources.extend(recording_utilities.get_recording_resources(
            [n_machine_time_steps *
             self.SDRAM_PER_TIMER_TICK_PER_RECORDING],
            self._receive_buffer_host, self._receive_buffer_port))

        return resources

    @overrides(AbstractHasAssociatedBinary.get_binary_file_name)
    def get_binary_file_name(self):
        return "pf_agg.aplx"

    @overrides(AbstractHasAssociatedBinary.get_binary_start_type)
    def get_binary_start_type(self):
        return ExecutableStartType.USES_SIMULATION_INTERFACE
        
    @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    def get_n_keys_for_partition(self, partition, graph_mapper):
        if partition.identifier == app_constants.EDGE_PARTITION_RE_SAMPLE:
            return self.KEYS_PER_SAMPLE_DATA
        if (partition.identifier ==
                app_constants.EDGE_PARTITION_TARGET_POSITION):
            return self.KEYS_PER_TARGET_POSITION
        raise Exception("Incorrect Partition Name at aggregator")

    @inject_items({"n_machine_time_steps": "TotalMachineTimeSteps"})
    @overrides(MachineDataSpecableVertex.generate_machine_data_specification,
               additional_arguments=["n_machine_time_steps"])
    def generate_machine_data_specification(
            self, spec, placement, machine_graph, routing_info, iptags,
            reverse_iptags, machine_time_step, time_scale_factor,
            n_machine_time_steps):
        self._placement = placement

        # Setup words + 1 for flags + 1 for recording size
        setup_size = constants.SYSTEM_BYTES_REQUIREMENT

        # Reserve SDRAM space for memory areas:
        edges = list(machine_graph.get_edges_ending_at_vertex(self))

        # Create the data regions for hello world
        self._reserve_memory_regions(spec, setup_size, len(edges))

        # write data for the simulation data item
        spec.switch_write_focus(self.DATA_REGIONS.SYSTEM.value)
        spec.write_array(simulation_utilities.get_simulation_header_array(
            self.get_binary_file_name(), machine_time_step,
            time_scale_factor))

        # recording data region
        spec.switch_write_focus(self.DATA_REGIONS.RECORDED_DATA.value)
        spec.write_array(recording_utilities.get_recording_header_array(
            [n_machine_time_steps *
             self.SDRAM_PER_TIMER_TICK_PER_RECORDING],
            self._time_between_requests, n_machine_time_steps *
            self.SDRAM_PER_TIMER_TICK_PER_RECORDING + 256, iptags))

        # writing my key
        spec.switch_write_focus(
            region=self.DATA_REGIONS.TRANSMISSION_DATA.value)
        
        key1 = routing_info.get_first_key_from_pre_vertex(
            self, app_constants.EDGE_PARTITION_RE_SAMPLE)
        if key1 is None:
            raise Exception("FUCKING IDIOT")
        if key1 is None:
            spec.write_value(0)
            spec.write_value(0)
        else:            
            spec.write_value(1)
            spec.write_value(key1)

        key2 = routing_info.get_first_key_from_pre_vertex(
            self, app_constants.EDGE_PARTITION_TARGET_POSITION)

        if key2 is None:
            spec.write_value(0)
            spec.write_value(0)
        else:
            spec.write_value(1)
            spec.write_value(key2)

        # writing if recording
        spec.switch_write_focus(region=self.DATA_REGIONS.CONFIG.value)
        if self._record_data:
            spec.write_value(1)
        else:
            spec.write_value(0)

        # write partner base key
        partner_edges = \
            machine_graph.get_outgoing_edge_partition_starting_at_vertex(
                self, app_constants.EDGE_PARTITION_RE_SAMPLE)
        partner_vertex = list(partner_edges.edges)[0].post_vertex
        partner_key = routing_info.get_first_key_from_pre_vertex(
            partner_vertex, app_constants.EDGE_PARTITION_PARTICLE_STATE)
        spec.write_value(partner_key)
        
        # writing particle keys
        spec.switch_write_focus(
            region=self.DATA_REGIONS.RECEPTION_BASE_KEYS.value)
        if len(edges) != self.n_particles:
            raise Exception("something wrong")
        spec.write_value(len(edges))
        for e in edges:
            edge_key = routing_info.get_first_key_for_edge(e)
            spec.write_value(edge_key)

        # End-of-Spec:
        spec.end_specification()

    def _reserve_memory_regions(self, spec, system_size, n_particles):
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.SYSTEM.value, size=system_size,
            label='systemInfo')
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.RECORDED_DATA.value,
            size=recording_utilities.get_recording_header_size(1),
            label="Recording")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.TRANSMISSION_DATA.value,
            size=self.TRANSMISSION_DATA_SIZE,
            label="My Key")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.RECEPTION_BASE_KEYS.value,
            size=4 + (n_particles * 4),
            label="Particle Keys")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.CONFIG.value,
            size=self.CONFIG_DATA_REGION_SIZE,
            label="Config (recording or not)")

    def get_data(self, placement, buffer_manager):
        """ Get the data written into sdram

        :param placement: the location of this vertex
        :param buffer_manager: the buffer manager
        :return: string output
        """
        data_pointer, missing_data = buffer_manager.get_data_for_vertex(
            placement, self.RECORDED_REGION_ID)
        if missing_data:
            raise Exception("missing data!")
        record_raw = data_pointer.read_all()
        output = str(record_raw)
        data = list()
        if len(output) != 0:
            formatstring = "<{}III".format(
                len(output) / self.SDRAM_PER_TIMER_TICK_PER_RECORDING)
            elements = struct.unpack(formatstring, bytes(output))
            for position in range(
                    0, len(output) / self.SDRAM_PER_TIMER_TICK_PER_RECORDING):
                x = elements[0 + (position * 3)]
                y = elements[1 + (position * 3)]
                r = elements[2 + (position * 3)]
                data.append((x, y, r))
        return data

    def get_minimum_buffer_sdram_usage(self):
        return self._string_data_size

    @inject_items({"n_machine_time_steps": "TotalMachineTimeSteps"})
    @overrides(AbstractReceiveBuffersToHost.get_n_timesteps_in_buffer_space,
               additional_arguments=["n_machine_time_steps"])
    def get_n_timesteps_in_buffer_space(
            self, buffer_space, machine_time_step, n_machine_time_steps):
        return recording_utilities.get_n_timesteps_in_buffer_space(
            buffer_space, [
                n_machine_time_steps *
                self.SDRAM_PER_TIMER_TICK_PER_RECORDING])

    def get_recorded_region_ids(self):
        return [self.RECORDED_REGION_ID]

    def get_recording_region_base_address(self, txrx, placement):
        return helpful_functions.locate_memory_region_for_placement(
            placement, self.DATA_REGIONS.RECORDED_DATA.value, txrx)
