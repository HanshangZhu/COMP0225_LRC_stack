# CMU-Exploration (Flagged) <!-- title -->
This repo is a flagged variant of CMU-Exploration with a Gazebo stack under active development. <!-- summary -->
The original CMU codebases are preserved as submodules. <!-- constraint -->
Use the Gazebo stack for simulation iteration and debugging. <!-- intent -->
 <!-- spacer -->
## Quick Start (Gazebo) <!-- section -->
1) Ensure the `cmu_env` environment exists (see Installation). <!-- step -->
2) Run: `./start_exploration.sh`. <!-- step -->
3) Open RViz to visualize frontier markers and goals. <!-- step -->
4) Observe robot motion in Gazebo. <!-- step -->
 <!-- spacer -->
## Installation <!-- section -->
1) Create/update the conda environment from `cmu_env.yml`: <!-- step -->
   - `micromamba env create -f cmu_env.yml` <!-- cmd -->
   - If it already exists: `micromamba env update -n cmu_env -f cmu_env.yml --prune` <!-- cmd -->
2) Build the workspace (from repo root): <!-- step -->
   - `source /opt/ros/humble/setup.bash` <!-- cmd -->
   - `colcon build --symlink-install` <!-- cmd -->
3) Load runtime environment in each shell: <!-- step -->
   - `source setup_cmu_env.bash` <!-- cmd -->
4) Launch simulation: <!-- step -->
   - `./start_exploration.sh` <!-- cmd -->
 <!-- spacer -->
## Project Layout <!-- section -->
- `src/go2_gazebo_sim/` — Gazebo simulation stack (launch, scripts, worlds). <!-- map -->
- `src/autonomy_stack_go2/` — CMU autonomy stack (submodule). <!-- map -->
- `src/unitree-go2-ros2/` — Unitree Go2 ROS 2 stack (submodule). <!-- map -->
- `cmu_env.yml` — pinned `micromamba` environment for this workspace. <!-- map -->
- `WALKTHROUGH.md` — detailed Gazebo stack walkthrough. <!-- map -->
 <!-- spacer -->
## Notes <!-- section -->
- This repo uses 2D lidar for exploration in Gazebo. <!-- note -->
- CMU planner integration requires 3D sensing; not enabled by default here. <!-- note -->
- Build artifacts are ignored via `.gitignore`. <!-- note -->
