import sys
from pathlib import Path
import rclpy
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.kit.commands
import omni.usd
from pxr import UsdGeom, UsdPhysics

stage_ctx = omni.usd.get_context()
stage_ctx.new_stage()
stage = stage_ctx.get_stage()

import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
ext_manager.set_extension_enabled_immediate("isaacsim.sensors.physx", True)
ext_manager.set_extension_enabled_immediate("isaacsim.sensors.physics", True)
ext_manager.set_extension_enabled_immediate("isaacsim.sensors.rtx", True)
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
ext_manager.set_extension_enabled_immediate("isaacsim.core.nodes", True)

for _ in range(3):
    omni.kit.app.get_app().update()

ok, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = False
import_config.create_physics_scene = False
import_config.import_inertia_tensor = True

workspace_root = Path(__file__).resolve().parents[3]
urdf_path = workspace_root / "install/go2_description/share/go2_description/urdf/go2_description.urdf"
ok, imported_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=str(urdf_path),
    import_config=import_config,
    get_articulation_root=True,
)
print(f"Imported path = {imported_path}")

arts = []
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        arts.append(str(p.GetPath()))
print(f"Found Articulation Roots: {arts}")

app.close()
