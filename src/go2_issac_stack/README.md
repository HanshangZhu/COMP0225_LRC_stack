# go2_issac_stack

Isaac Sim/Lab migration package for the CMU Go2 exploration stack.

This package keeps existing autonomy nodes from `go2_gazebo_sim` and adds:

- an Isaac topic compatibility bridge (`isaac_topic_router.py`)
- dual-robot Isaac launch orchestration
- local nav/SLAM config copies for Isaac tuning

## Full installation guide (`cmu_isaac`)

Use the dedicated environment file `cmu_isaac.yml` at workspace root. It is
generated from the **actual** `conda list` output of `cmu_isaac` and includes
both conda and pip-installed packages for Isaac workflows.

### 1) Create/update environment from full dependency file

From workspace root:

```bash
cd /home/hz/cmu_exploration_ws
```

Create (first time):

```bash
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda env create -f cmu_isaac.yml
```

Update existing env to match file exactly:

```bash
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda env update -n cmu_isaac -f cmu_isaac.yml --prune
```

### 2) Activate environment and ROS

```bash
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda activate cmu_isaac
source /opt/ros/humble/setup.bash
```

### 3) Build workspace

```bash
cd /home/hz/cmu_exploration_ws
colcon build --packages-select go2_issac_stack
source install/setup.bash
```

### 4) Verify critical Python dependency availability

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

### 5) Launch (dual, GUI + RViz)

```bash
ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
  isaac_headless:=false \
  rviz:=true
```

### 6) Launch with official Unitree USD

```bash
ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
  isaac_headless:=false \
  rviz:=true \
  isaac_lidar_mode:=rtx \
  isaac_robot_usd:=/home/hz/cmu_exploration_ws/src/go2_issac_stack/assets/unitree_model/Go2/usd/go2.usd
```

Optional when lidar prim is known:

```bash
isaac_robot_lidar_prim:=<absolute_or_relative_lidar_prim_path>
```

### 7) Full dependency list source

The authoritative full dependency list is in:

- `cmu_isaac.yml`

You can inspect it directly:

```bash
sed -n '1,260p' /home/hz/cmu_exploration_ws/cmu_isaac.yml
```

### 8) Refresh `cmu_isaac.yml` from current env (after installs/updates)

```bash
source /home/hz/miniforge3/etc/profile.d/conda.sh
conda activate cmu_isaac
conda list > /home/hz/cmu_exploration_ws/.cmu_isaac_conda_list.txt
```

Then regenerate the YAML from that terminal list:

```bash
python3 - <<'PY'
from pathlib import Path
import re

src = Path('/home/hz/cmu_exploration_ws/.cmu_isaac_conda_list.txt')
out = Path('/home/hz/cmu_exploration_ws/cmu_isaac.yml')

conda_deps, pip_deps = [], []
for line in src.read_text().splitlines():
    if not line.strip() or line.startswith('#'):
        continue
    parts = re.split(r'\\s+', line.strip())
    if len(parts) < 4:
        continue
    name, version, build, channel = parts[0], parts[1], parts[2], parts[3]
    if channel == 'pypi' or build == 'pypi_0':
        pip_deps.append(f'{name}=={version}')
    else:
        conda_deps.append(f'{name}={version}={build}')

conda_deps = sorted(dict.fromkeys(conda_deps))
pip_deps = sorted(dict.fromkeys(pip_deps), key=str.lower)

lines = ['name: cmu_isaac', 'channels:', '  - conda-forge', 'dependencies:']
lines += [f'  - {d}' for d in conda_deps]
if pip_deps:
    lines.append('  - pip:')
    lines += [f'    - {p}' for p in pip_deps]
out.write_text('\\n'.join(lines) + '\\n')
print(f'Wrote {out} | conda={len(conda_deps)} pip={len(pip_deps)}')
PY
```

## Launch

```bash
ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py
```

By default this launch now starts an internal dual-robot Isaac bringup script
(`isaac_t_world_dual_bringup.py`) with namespaces `go2_1` and `go2_2`.

## Single Robot `t_world` readiness run

```bash
ros2 launch go2_issac_stack single_go2_t_world_frontier.launch.py
```

By default this launch now starts an internal Isaac bringup script that:

- loads `t_dual_corridor.world` wall geometry
- imports `go2_description/urdf/go2_description.urdf`
- publishes `/clock`, `/<ns>/odom`, `/<ns>/imu`, `/<ns>/lidar/points`
- consumes `/<ns>/isaac/cmd_vel`

Override with a custom command when needed:

```bash
ros2 launch go2_issac_stack single_go2_t_world_frontier.launch.py \
  start_isaac_sim:=true \
  isaac_sim_command:='python3 /abs/path/to/custom_isaac_bringup.py --headless'
```

When `run_readiness_gate:=true` (default), the `readiness_gate.py` node exits:

- `0` when required streams are healthy and exploration progresses
- `1` on timeout/stale/freeze conditions

Pass criterion uses bounded-map coverage from closed walls, default:

- `readiness_required_coverage:=0.80`

Automation script (build + source + launch + log + PASS/FAIL detection):

```bash
bash src/go2_issac_stack/scripts/run_single_t_world_readiness.sh
```

The script exits with:

- `0` when `READINESS PASS` appears in launch output
- `1` when `READINESS FAIL` appears
- `124` on script timeout waiting for a pass/fail marker

## Start Isaac from launch (optional)

```bash
ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
  isaac_headless:=true \
  isaac_lidar_mode:=rtx \
  use_shared_map:=false
```

Override with a custom simulator command if needed:

```bash
ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
  isaac_sim_command:='python3 /abs/path/to/custom_isaac_bringup.py --headless'
```

## Notes

- Namespace defaults are `go2_1` and `go2_2`.
- Isaac is expected to publish `/<ns>/lidar/points`, `/<ns>/imu`, and `/<ns>/odom`.
- Command output for control is `/<ns>/isaac/cmd_vel`.
