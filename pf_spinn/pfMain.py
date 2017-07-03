"""
Hello World program on Spinnaker

Each core stores into its region in SDRAM the string:
"Hello World from $chip.x, $chip.y, $core"

We then fetch the written data and print it on the python console.
"""

import spinnaker_graph_front_end as front_end

from pfParticle.pfParticle_Vertex import pfParticle_Vertex
from pfParticle.pfAgg_Vertex import pfAgg_Vertex
from pacman.model.graphs.machine import MachineEdge


import logging

logger = logging.getLogger(__name__)

n_chips_required = None
if front_end.is_allocated_machine():
    n_chips_required = 4
front_end.setup(n_chips_required=n_chips_required)

machine = front_end.machine()

machine_time_step = 1000
time_scale_factor = 1
n_particles = 100

# calculate total number of 'free' cores for the given board
# (i.e. does not include those busy with SARK or reinjection)
#total_number_of_cores = len([
#    processor for chip in machine.chips for processor in chip.processors
#    if not processor.is_monitor])


#VERTICES

particlelist = list()
agglist = list()

# create particles
for x in range(0, n_particles):
    vertex = pfParticle_Vertex(x = rand(304), y = rand(240), r = rand(30), tw = 10000)
    front_end.add_machine_vertex_instance(
        vertex,
        label="Particle {}".format(x))
    particlelist.append(vertex)

#create aggrigators   
for x in range(0, n_particles):
    vertex = pfAgg_Vertex(record_data = (x == 0), n_particles = n_particles)
    front_end.add_machine_vertex_instance(
        vertex,
        label="Aggrigator {}".format(x))
    agglist.append(vertex)

#create "input" 
#running with test data use this vertex
#input_vertex = ReverseIPTagMulticastSourceMachineVertex()
#front_end_add_machine_vertex_instance(
#    input_vertex,
#    label="Input Vertex")

#when running on the icub we'll need this vertex
input_vertex = icub_in_vertex(spinnaker_link_id = 1, board_address = None)
front_end_add_machine_vertex_instance(
    input_vertex,
    label="Input Vertex")

output_vertex = icub_out_vertex(spinnaker_link_id = 1, board_address = None)
front_end_add_machine_vertex_instance(
    output_vertex,
    label="Output Vertex")

#EDGES

for x in range(0, n_particles):
    front_edge.add_machine_edge_instance(
        MachineEdge(input_vertex, particlelist[x], label="Edge Input to P{}".format(x)),
        "Event")
        
for x in range(0, n_particles):
    for y in range(0, n_particles):
        front_end.add_machine_edge_instance(
            MachineEdge(particlelist[x], agglist[y], label="Edge P{} to A{}".format(x, y)),
            "Particle State")


for x in range(0, n_particles):
    front_edge.add_machine_edge_instance(
        MachineEdge(agglist[x], particlelist[x], label="Edge A{} to P{}".format(x, x)),
        "Resample Data")

#used with the icub
front_end.add_machine_edge_instance(
    MachineEdge(agglist[0], output_vertex, label="Final Result Edge"),
    "Target Position")




front_end.run(10)

#used with test data
placements = front_end.placements()
buffer_manager = front_end.buffer_manager()
placement = placements.get_placement_of_vertex(agglist[0])
data = agglist[0].getdata(placement, buffer_manager)
print(data)

front_end.stop()
