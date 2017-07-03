"""
Hello World program on Spinnaker

Each core stores into its region in SDRAM the string:
"Hello World from $chip.x, $chip.y, $core"

We then fetch the written data and print it on the python console.
"""

import spinnaker_graph_front_end as front_end


import logging

from spinnaker_graph_front_end.examples.hello_world.hello_world_vertex import \
    HelloWorldVertex
from spinnaker_graph_front_end.examples import hello_world

logger = logging.getLogger(__name__)

n_chips_required = None

# needs to have no () sas its not a property
if front_end.is_allocated_machine:
    n_chips_required = 2

# need to say where to get binary from. just using hello world as a plug in
front_end.setup(n_chips_required=n_chips_required,
                model_binary_module=hello_world)

# needs at least one vertex to run
front_end.add_machine_vertex(
            HelloWorldVertex,
            {},
            label="Hello World at x ".format())
front_end.run(60000)

front_end.stop()
