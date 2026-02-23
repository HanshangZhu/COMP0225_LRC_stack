# Current Problems: RTX Lidar Troubleshooting

## Issue: Empty Point Clouds (width=0)
The RTX Lidar in Isaac Sim publishes to ROS 2 correctly, but every message contains zero points (`width=0`). This occurs despite the walls rendering correctly in the simulation viewport and being configured as meshes with ray-trace compatible materials.

### Attempted Fixes and Results

| Attempt | Change Made | Rationale | Result |
| :--- | :--- | :--- | :--- |
| **1. ROS 2 Graph** | Added `ROS2Context` node. | The `ROS2RtxLidarHelper` node needs a context to establish a ROS publisher. | **Topic active** but data is empty. |
| **2. Materials** | `UsdPreviewSurface` -> `OmniPBR`. | RTX Lidar requires MDL materials; native USD shaders are often "invisible" to it. | Still empty. |
| **3. Renderer** | `HydraStorm` -> `RayTracedLighting`. | RTX sensors require the ray-tracing renderer to build acceleration structures (BVH). | Still empty. |
| **4. Geometry** | `UsdGeom.Cube` -> `UsdGeom.Mesh`. | Procedural cubes are sometimes not tessellated into the ray-tracing BVH. Meshes are explicit. | **Walls visible in UI**, but Lidar still empty. |
| **5. Extensions** | Enabled `omni.sensors.nv.lidar` etc. | Specialized RTX sensor pipelines require specific extensions to be initialized. | Still empty. |
| **6. Warmup** | 10 frames -> 50 frames. | RTX render products often need more time to initialize the buffer than CPU sensors. | Still empty. |
| **7. Config** | Forced `Example_Rotary` config. | Isolate whether our custom `Unitree_L1.json` was the source of the failure. | Still empty. |

### Current Status & Next Steps
- **Diagnosis Script**: An inline raw diagnostic has been added to `isaac_t_world_dual_bringup.py` that reads the render product directly via a Replicator annotator.
- **Goal**: Determine if the "Kill Zone" is at the **Isaac Sim Sensor level** (annotator empty) or the **ROS Relay level** (annotator has data, but ROS topic is empty).
- **Action Required**: Restart simulation and check terminal logs for `[RTX DIAG]` tags.
