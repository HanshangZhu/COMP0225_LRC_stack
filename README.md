# CMU Exploration Workspace (Isaac Stream)

Primary workflow in this repo is the `cmu_isaac` stream.

## Quick Start
1. Create or update the `cmu_isaac` environment from `cmu_isaac.yml`.
2. Build the workspace with `colcon`.
3. Run `run_exploration_issac.sh` using either autonomy baseline (synthetic GT pointcloud) or sensor realism (RTX lidar).

## Environment Setup (`cmu_isaac`)
```bash
cd /home/hz/cmu_exploration_ws
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda env create -f cmu_isaac.yml
```

If env already exists:
```bash
cd /home/hz/cmu_exploration_ws
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda env update -n cmu_isaac -f cmu_isaac.yml --prune
```

## Build
```bash
cd /home/hz/cmu_exploration_ws
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda activate cmu_isaac
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_issac_stack
source install/setup.bash
```

## External Repositories
- FAST-LIO (ROS2 fork used in this workspace): `https://github.com/Ericsii/FAST_LIO_ROS2`
- Unitree Go2 ROS2 stack: `https://github.com/anujjain-dev/unitree-go2-ros2.git`
- Unitree USD/model assets: `https://huggingface.co/datasets/unitreerobotics/unitree_model`

If cloning this workspace fresh, pull submodules too:
```bash
git clone --recurse-submodules <MAIN_REPO_URL>
```

If already cloned:
```bash
git submodule update --init --recursive
```

## Run (Shell Profiles)
```bash
cd /home/hz/cmu_exploration_ws
bash run_exploration_issac.sh autonomy_baseline
```

```bash
cd /home/hz/cmu_exploration_ws
bash run_exploration_issac.sh sensor_realism
```

Available profiles:
- `autonomy_baseline`: CPU synthetic raycast pointcloud (GT geometry), stable autonomy baseline.
- `sensor_realism`: native RTX lidar stream, realism branch for sensor validation.
- `debug`: higher-fidelity debug profile.

Aliases:
- `balanced` -> `autonomy_baseline`
- `perf` -> `sensor_realism`

## Notes
- Main Isaac launch lives in `go2_issac_stack`.
- Perception source is swappable while downstream mapping/frontier/controller stays unchanged.
- For detailed package internals, see `src/go2_issac_stack/README.md`.
