"""
Hello World program on Spinnaker

Each core stores into its region in SDRAM the string:
"Hello World from $chip.x, $chip.y, $core"

We then fetch the written data and print it on the python console.
"""

import logging
# common import
import random

# import front end
import spinnaker_graph_front_end as front_end
# import graph components
from pacman.model.graphs.machine import MachineEdge
# import for binary location
from pf_spinn import binaries
# constants
from pf_spinn import constants
from pf_spinn.ICUB_input_vertex.ICUB_input_vertex import ICUBInputVertex
from pf_spinn.ICUB_output_vertex.ICUB_output_vertex import ICUBOutputVertex
from pf_spinn.pf_agg.pf_agg_vertex import PfAggVertex
from pf_spinn.pf_particle.pf_particle_vertex import PfParticleVertex
from pf_spinn.roi_filter.roi_filter_vertex import RetinaFilter
from read_dataset import load_spike_train
from spinn_front_end_common.utility_models. \
    reverse_ip_tag_multicast_source_machine_vertex import \
    ReverseIPTagMulticastSourceMachineVertex

# logger!
logger = logging.getLogger(__name__)

# get machine and setup backend
n_chips_required = 4
front_end.setup(n_chips_required=n_chips_required,
                model_binary_module=binaries)
#machine = front_end.machine()
#machine_ip = machine.ethernet_connected_chips[0].ip_address
machine_ip = "192.168.2.201"

# state variables
use_spinn_link = False
machine_time_step = 1000
time_scale_factor = 1
n_particles = 20

spinnaker_link_used = 0
packets_threshold = 30

# calculate total number of 'free' cores for the given board
# (i.e. does not include those busy with SARK or reinjection)
# total_number_of_cores = len([
#    processor for chip in machine.chips for processor in chip.processors
#    if not processor.is_monitor])

filename = "data.log.spiking.txt"
spike_train = load_spike_train(filename)

# VERTICES
particle_list = list()
agg_list = list()
filter_list = list()

# create particles
main_particle = True
the_main_particle = None
for x in range(0, n_particles):
    vertex = PfParticleVertex(
        x=random.randint(0, 304), y=random.randint(0, 240),
        r=random.randint(0, 30), packet_threshold=packets_threshold,
        label="Particle {}".format(x), id=x, main_particle=main_particle)

    if main_particle:
        the_main_particle = vertex
        main_particle = False

    front_end.add_machine_vertex_instance(vertex)
    particle_list.append(vertex)

# create aggregator
for x in range(0, n_particles):
    vertex = PfAggVertex(
        record_data=(x == 0), n_particles=n_particles,
        label="Aggrigator {}".format(x))
    front_end.add_machine_vertex_instance(vertex)
    agg_list.append(vertex)

# create "input"
# running with test data use this vertex
if use_spinn_link:
    # when running on the icub we'll need this vertex
    input_vertex = ICUBInputVertex(
        spinnaker_link_id=spinnaker_link_used, board_address=None,
        label="Input Vertex")
    front_end.add_machine_vertex_instance(input_vertex)
else:
    input_vertex = ReverseIPTagMulticastSourceMachineVertex(
        board_address=machine_ip, receive_port=12347,
        reserve_reverse_ip_tag=True, virtual_key=constants.RETINA_BASE_KEY,
        buffer_notification_ip_address="0.0.0.0",
        n_keys=1048575,
        label="Input Vertex", send_buffer_times=spike_train)
    front_end.add_machine_vertex_instance(input_vertex)

# create retina filters and edges from retina to filters
for y_row in range(0, constants.RETINA_Y_SIZE):
    partition_identifier = "retina_slice_row_{}".format(y_row)
    vertex = RetinaFilter(
        partition_identifier=partition_identifier, filter=y_row,
        row_id=y_row, atoms_in_row=constants.RETINA_Y_SIZE)
    filter_list.append(vertex)
    front_end.add_machine_vertex_instance(vertex)
    front_end.add_machine_edge_instance(
        MachineEdge(input_vertex, vertex,
                    "Edge between retina and filter"),
        partition_identifier)

# create edge from main particle to filters
for filter_vertex in filter_list:
    front_end.add_machine_edge_instance(
        MachineEdge(the_main_particle, filter_vertex),
        constants.EDGE_PARTITION_PARTICLE_TO_FILTER)

output_vertex = ICUBOutputVertex(
    spinnaker_link_id=spinnaker_link_used, board_address=None,
    label="Output Vertex")
front_end.add_machine_vertex_instance(output_vertex)

# EDGES from filter to particles
for x in range(0, n_particles):
    for filter_vertex in filter_list:
        front_end.add_machine_edge_instance(
            MachineEdge(
                filter_vertex, particle_list[x],
                label="Edge Input to P{}".format(x)),
            constants.EDGE_PARTITION_EVENT)
        
for x in range(0, n_particles):
    for y in range(0, n_particles):
        front_end.add_machine_edge_instance(
            MachineEdge(
                particle_list[x],
                agg_list[y], label="Edge P{} to A{}".format(x, y)),
            constants.EDGE_PARTITION_PARTICLE_STATE)


for x in range(0, n_particles):
    front_end.add_machine_edge_instance(
        MachineEdge(
            agg_list[x],
            particle_list[x],
            label="Edge A{} to P{}".format(x, x)),
        constants.EDGE_PARTITION_RE_SAMPLE)

# used with the icub
front_end.add_machine_edge_instance(
    MachineEdge(
        agg_list[0],
        output_vertex,
        label="Final Result Edge"),
    constants.EDGE_PARTITION_TARGET_POSITION)

front_end.run(20000)

# used with test data
placements = front_end.placements()
buffer_manager = front_end.buffer_manager()
placement = placements.get_placement_of_vertex(agg_list[0])
data = agg_list[0].get_data(placement, buffer_manager)
print(data)

front_end.stop()
