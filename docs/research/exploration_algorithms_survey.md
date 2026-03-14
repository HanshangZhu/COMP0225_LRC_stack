# Exploration Algorithms Survey (2020–2025)

A landscape overview of SOTA, baseline, and popular exploration planners relevant to the COMP0225 LRC stack (ground robots, 2D/3D, single & multi-robot).

---

## 1. Classical Frontier-Based (Baseline)

The workhorse. Identify boundaries between known-free and unknown cells, rank by information gain / distance, drive to the best one.

| Variant | Year | Key Idea | Open Source |
|---|---|---|---|
| **Yamauchi frontier** | 1997 | Original greedy closest-frontier | Widely reimplemented |
| **Cost-utility frontier** | 2000s | Weigh gain vs travel cost | Nav2 `explore_lite` (ROS2) |
| **RRT-based frontier** | 2020+ | Sample-based frontier detection in large maps | Various |
| **IEFA** (Internal & External Frontier) | 2025 | Separates internal/external frontiers for better target selection | Paper only |
| **GFGE** (Grid Flex-Graph) | 2024 | Quad-grid + graph for efficient local/global mapping | Paper only |

> **Your stack**: `cfpa2_collaborative_autonomy` uses geometric frontier detection + joint utility scoring. `go2_nav_algorithms` has `simple_scan_mapper_cpp` feeding the frontier detector.

---

## 2. CMU Autonomy Stack Family

Dominant in subterranean / field robotics. DARPA SubT lineage.

### TARE (2021)
- **Hierarchical 2-layer**: coarse global + detailed local
- 80% more explored volume/s vs prior SOTA, <50% computation
- Handles complex 3D (caves, tunnels, multi-story)
- **GitHub**: `caochao39/tare_planner` (ROS1), your repo has `go2_tare_planner_ros2`

### FAR Planner (2022)
- **Dynamic visibility graph** for fast route replanning (ms-scale)
- 12–47% less travel time than A*, D\* Lite, RRT\*
- Not an exploration planner per se — it's a **goal-directed route planner** paired with TARE
- **GitHub**: `MichaelFYang/far_planner`, your repo has `go2_far_planner`

### GBPlanner / GBPlanner2 (2020–2021)
- **Graph-based**, RRG local + global repositioning to frontiers
- Strong SubT baseline, used by many teams
- Can be compute-heavy in large environments
- **GitHub**: `ntnu-arl/gbplanner_ros`

### M-TARE / MUI-TARE (2023)
https://github.com/caochao39/mtare_planner.git
- Multi-robot extension of TARE
- Coarse global map sharing + "pursuit" coordination under comm constraints
- Handles unknown initial poses
- **GitHub**: `caochao39/mtare_planner` (Docker, preliminary release Dec 2023)
- Your repo has `mtare_ros2` + `mtare_ros1_ws` (ROS1 bridge)

---

## 3. HKUST / Fast-Planner Family

### FUEL (2021)
- **Frontier Information Structure (FIS)** + hierarchical coverage planner
- Designed for UAVs but applicable to ground
- Multi-UAV code released Feb 2023
- **GitHub**: `HKUST-Aerial-Robotics/FUEL`

### RACER (2022)
- Rapid exploration with hierarchical planning
- Dual-resolution (coarse tour + fine trajectory), real-time capable
- **GitHub**: `HKUST-Aerial-Robotics/RACER`

---

## 4. Next-Best-View (NBV) Planners

Sample candidate viewpoints, evaluate expected information gain, go to best one. More common in 3D inspection/reconstruction.

| Method | Year | Key Idea |
|---|---|---|
| **AEP** (Autonomous Exploration Planner) | 2019 | RRT-based NBV with birdeye global |
| **SEE** (Surface Edge Explorer) | 2022 | Direct NBV from sensor measurements, no rigid voxel grid |
| **Shadowcasting NBV** | 2022 | Efficient gain calc for aerial robots |
| **LAGS** | 2023 | Local-and-Global Strategy, Bayesian + DRL to avoid regional legacy issues |

---

## 5. Learning-Based Exploration

Growing area but **sim-to-real transfer** and **sample efficiency** remain hard problems.

| Method | Year | Approach | Status |
|---|---|---|---|
| **ANS** (Active Neural SLAM) | 2020 | Learned global + local policy on top of neural map | Paper + code |
| **NeRF-based exploration** | 2023 | Use neural radiance fields for map representation | Research |
| **DRL + frontier selection** | 2024 | DRL selects which frontier to visit (not low-level control) | Multiple papers |
| **Dueling DDQN Active SLAM** | 2024 | Dueling Double DQN for viewpoint selection in Active SLAM | Paper |
| **Transformer-based multi-agent** | 2024 | Attention-based coordination for multi-robot exploration | Paper |

> **Practical assessment**: Learning-based methods generally underperform classical planners in structured indoor environments and require heavy training infrastructure. Best suited as a research direction rather than a production baseline right now.

---

## 6. Multi-Robot Coordination Strategies

| Strategy | Year | Key Idea | Code |
|---|---|---|---|
| **Greedy nearest frontier** | Baseline | Each robot goes to its closest frontier | Nav2 explore_lite |
| **Hungarian / auction-based** | Classic | Optimal assignment of frontiers to robots | Various |
| **CFPA2** (your stack) | 2024 | Joint utility maximization with overlap penalties + space-time A\* | `cfpa2_collaborative_autonomy` |
| **M-TARE** | 2023 | Pursuit-based coordination, sparse global sharing | `caochao39/mtare_planner` |
| **DMCE** (Decentralized Monte Carlo) | 2023 | MCTS-based, point-to-point comm, 30% faster than greedy | Paper + code |
| **CME-SSA** | 2024 | Hybrid deterministic + Salp Swarm meta-heuristic | Paper |
| **GBP distributed** | 2023 | Gaussian Belief Propagation for consensus | Paper |

---

## 7. Popular Open-Source Repos (ROS2-compatible or portable)

| Repo | Algorithms | ROS | Stars (approx) |
|---|---|---|---|
| [explore_lite](https://github.com/robo-friends/m-explore-ros2) | Frontier-based, multi-robot map merge | ROS2 | ~200 |
| [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) | FIS + hierarchical | ROS1 (portable) | ~700 |
| [tare_planner](https://github.com/caochao39/tare_planner) | TARE | ROS1 | ~600 |
| [far_planner](https://github.com/MichaelFYang/far_planner) | Visibility graph | ROS1 (portable) | ~200 |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) | GBPlanner2 | ROS1 | ~400 |
| [RACER](https://github.com/HKUST-Aerial-Robotics/RACER) | Rapid exploration | ROS1 | ~300 |
| [CMU autonomy_stack_diag](https://github.com/jizhang-cmu/autonomy_stack_diag_env) | Full CMU stack (TARE+FAR+terrain) | ROS2 | ~100 |

---

## 8. Relevance to Your Stack

| What you have | Category | Notes |
|---|---|---|
| `cfpa2_collaborative_autonomy` | Multi-robot coordination | Custom joint-maximization, overlap penalties |
| `go2_tare_planner_ros2` | TARE (single robot) | ROS2 port from CMU |
| `go2_far_planner` | FAR route planner | ROS2 port from CMU |
| `mtare_ros2` / `mtare_ros1_ws` | M-TARE (multi-robot) | Alternative coordinator to CFPA2 |
| Geometric frontier detection | Classic frontier | In `go2_nav_algorithms` |
| `reactive_nav` A\* planner | Local + global nav | C++ ctypes A\* for speed |

**Gaps / potential additions**:
- **Learning-based frontier selection** on top of your existing frontier detector (low risk, potential gain)
- **RACER-style dual-resolution** planning if you move to 3D volumetric exploration
- **DMCE** as an alternative decentralized coordinator (no central node needed)
- **NBV inspection** if the task evolves beyond coverage to semantic understanding
