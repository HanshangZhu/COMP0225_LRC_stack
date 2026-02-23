from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from pxr import Gf, UsdGeom
stage_ctx = omni.usd.get_context()
stage_ctx.new_stage()
stage = stage_ctx.get_stage()

import omni.kit.commands
usd_path = "/home/hz/cmu_exploration_ws/src/go2_issac_stack/assets/unitree_model/Go2/usd/go2.usd"

for i, ns in enumerate(["go2_1", "go2_2"]):
    target_path = f"/World/{ns}"
    omni.kit.commands.execute("CreateReferenceCommand", usd_context=stage_ctx, path_to=target_path, asset_path=usd_path, instanceable=False)
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(target_path))
    trans = xform.AddTranslateOp()
    trans.Set(Gf.Vec3d(i*2.0, 0, 0.5))

print("Created two robots.")
app.close()
