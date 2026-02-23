from isaacsim import SimulationApp
sim = SimulationApp({'headless': True})

import omni.graph.core as og
import omni.kit.app

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

sim.update()

types = og.get_all_node_types()
for t in types:
    if "ros2" in t.lower() or "joint" in t.lower() or "trajectory" in t.lower():
        print(t)

sim.close()
