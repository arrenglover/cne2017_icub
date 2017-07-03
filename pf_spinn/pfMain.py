"""
Hello World program on Spinnaker

Each core stores into its region in SDRAM the string:
"Hello World from $chip.x, $chip.y, $core"

We then fetch the written data and print it on the python console.
"""

import logging

# import for binary location
from pf_spinn import binaries

# import front end
import spinnaker_graph_front_end as front_end

# import graph components
from pacman.model.graphs.machine import MachineEdge
from pf_spinn.pfAgg.PfAggVertex import PfAggVertex
from pf_spinn.pfParticle.PfParticleVertex import PfParticleVertex
from pf_spinn.ICUB_input_vertex.ICUB_input_vertex import ICUBInputVertex
from pf_spinn.ICUB_output_vertex.ICUB_output_vertex import ICUBOutputVertex

# common import
import random

# logger!
logger = logging.getLogger(__name__)

# get machine and setup backend
n_chips_required = None
if front_end.is_allocated_machine():
    n_chips_required = 4
front_end.setup(n_chips_required=n_chips_required,
                model_binary_module=binaries)
machine = front_end.machine()

# state variables
machine_time_step = 1000
time_scale_factor = 1
n_particles = 100

# calculate total number of 'free' cores for the given board
# (i.e. does not include those busy with SARK or reinjection)
# total_number_of_cores = len([
#    processor for chip in machine.chips for processor in chip.processors
#    if not processor.is_monitor])


# VERTICES
particle_list = list()
agg_list = list()

# create particles
for x in range(0, n_particles):
    vertex = PfParticleVertex(
        x=random.random(304), y=random.random(240),
        r=random.random(30), label="Particle {}".format(x))
    front_end.add_machine_vertex_instance(vertex)
    particle_list.append(vertex)

# create aggrigators
for x in range(0, n_particles):
    vertex = PfAggVertex(
        record_data=(x == 0), transmit_target_position=(x == 0),
        n_particles=n_particles,
        label="Aggrigator {}".format(x))
    front_end.add_machine_vertex_instance(vertex)
    agg_list.append(vertex)

# create "input"
# running with test data use this vertex
# input_vertex = ReverseIPTagMulticastSourceMachineVertex()
# front_end_add_machine_vertex_instance(
#    input_vertex,
#    label="Input Vertex")

# when running on the icub we'll need this vertex
input_vertex = ICUBInputVertex(
    spinnaker_link_id=1, board_address=None, label="Input Vertex")
front_end.add_machine_vertex_instance(input_vertex)

output_vertex = ICUBOutputVertex(
    spinnaker_link_id=1, board_address=None, label="Output Vertex")
front_end.add_machine_vertex_instance(output_vertex)

# EDGES
for x in range(0, n_particles):
    front_end.add_machine_edge_instance(
        MachineEdge(
            input_vertex, particle_list[x],
            label="Edge Input to P{}".format(x)),
        "Event")
        
for x in range(0, n_particles):
    for y in range(0, n_particles):
        front_end.add_machine_edge_instance(
            MachineEdge(
                particle_list[x],
                agg_list[y], label="Edge P{} to A{}".format(x, y)),
            "Particle State")


for x in range(0, n_particles):
    front_end.add_machine_edge_instance(
        MachineEdge(
            agg_list[x],
            particle_list[x],
            label="Edge A{} to P{}".format(x, x)),
        "Resample Data")

#used with the icub
front_end.add_machine_edge_instance(
    MachineEdge(
        agg_list[0],
        output_vertex,
        label="Final Result Edge"),
    "Target Position")

front_end.run(10)

# used with test data
placements = front_end.placements()
buffer_manager = front_end.buffer_manager()
placement = placements.get_placement_of_vertex(agg_list[0])
data = agg_list[0].get_data(placement, buffer_manager)
print(data)

front_end.stop()
