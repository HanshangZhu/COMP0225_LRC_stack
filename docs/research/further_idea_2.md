# Further Idea 2: Physics-Informed Receding-Horizon Multi-Robot Exploration

一、 你的系统当前现状 (Current Status)
坦率地说，你现在的工程起点已经很扎实，但需要把“当前系统形态”描述得更准确。

工程基础： 当前主线是 Gazebo-first 双 Go2 协同探索；`run_cfpa2_gazebo.sh` 默认进入 `dual_go2_modular.launch.py`，并使用 `profile:=mtare_ros2` + `planner_backend:=cfpa2`。感知、建图、前沿提取、协调分配、执行控制和观测链路都已打通。

架构形态（纠正）： 你当前默认路径并不是“完全去中心化拍卖”。在 `cfpa2` 路径下，核心是中心化协调器做双机联合打分与联合分配（IG - 路径代价 - 切换惩罚 + 重叠惩罚），并叠加了 stuck recovery、近距离停车仲裁、space-time A* 短时避冲突、shared-map fail-open 等稳健机制。

现状总结： 你的机器人集群目前是“几何驱动、语义缺位”的强工程基座。它们已经具备稳健协同与安全执行能力，但高层语义理解、可解释决策与语义任务规划仍是下一步创新切入点。



## One-Liner
Replace CFPA2's myopic frontier utility with a receding-horizon planner that uses a shared physics-informed neural network to predict both self and teammate trajectories, evaluating information gain on simulated future maps.

## Problem
Current CFPA2 evaluates each frontier at t=now only, and predicts teammates with constant-velocity straight lines. This fails when:
- Robots turn sharply at junctions (linearization error)
- Robots slip on rough terrain (non-Gaussian dynamics)
- One robot blocks a bottleneck the other needs (no intent awareness)

EKF-based state estimators break under these nonlinearities. Myopic utility can't see that a dead-end wastes future exploration time.

## Proposed Method

### 1. Shared Physics-NN (Self + Teammate)
```
x_{t+1} = f_physics(x_t) + f_residual_NN(x_t, goal, local_map_8×8)
           ├─ known ──────┘  └─ learned (~50K params) ────────────┘
```
- Same model, same weights for both robots — a robot is a robot
- Learns residuals over unicycle kinematics: obstacle slowdown, turn anticipation, slip recovery
- Input: (x, y, θ, v, ω, goal, 8×8 map patch) → Output: (Δx, Δy, Δθ) residual
- Inference: < 0.5 ms CPU

### 2. Multi-Step IG Rollout (K=3 steps)
```
For each candidate frontier f₀:
  sim_map = current_map
  for k in 0..K:
    self_pose[k]  = physics_nn.predict(self, f₀, sim_map)
    mate_pose[k]  = physics_nn.predict(teammate, their_goal, sim_map)
    simulate_observation(sim_map, self_pose[k])   # reveal unknown cells
    simulate_observation(sim_map, mate_pose[k])   # teammate also reveals
    U += γ^k * IG(sim_map, self_pose[k])
    f₀ = best_next_frontier(sim_map)  # greedy re-eval on predicted map
  return U
```

### 3. Joint Assignment
```
(f*_a, f*_b) = argmax rollout_U_a(f_a) + rollout_U_b(f_b)
```
Overlap avoidance is **implicit** — teammate's map reveal is already simulated, so double-counting IG is impossible.

## Why Physics-NN > EKF / Constant-Velocity

| Failure mode | EKF / const-vel | Physics-NN |
|---|---|---|
| Sharp turn at junction | Linearization explodes | Map patch shows junction → predicts turn |
| Slip on rough terrain | Gaussian noise can't model bimodal | Learns slip patterns from data |
| Collision avoidance swerve | Violates small-perturbation | Trained on near-obstacle trajectories |
| Teammate changes intent | No concept of "goal" | Goal is explicit input |

## Compute Budget
~20 ms per tick at 1 Hz. 50 frontiers × 3 steps × (NN inference + map simulation + IG query). Well within budget on Jetson.

## Training
- Collect 50+ Gazebo trials → extract (state, goal, map_patch, actual_next_state) tuples
- Train on residual: `L = ||Δ_pred − Δ_actual||² + λ·||f_residual||²`
- Regularize toward zero residual = trust physics by default

## Novelty (Gap in Literature)
No existing work combines ALL of:
1. Receding-horizon utility for exploration (not just navigation)
2. Physics-informed learned trajectory prediction
3. Joint future map simulation for multi-robot coordination
4. Real-world deployment on legged robots

Each exists independently; the intersection is empty.

## Publication Target
- **ICRA 2026/2027** (best fit: systems + learning + real hardware)
- **CoRL 2026** (if learned observer is main contribution)
- **IROS 2026** (strong alternative)

## Key Experiments Needed
1. **Sim**: ≥ 3 envs × 20 trials — myopic vs. K={1,3,5} × {const-vel, A*, physics-NN}
2. **Real**: 2× Go2, ≥ 2 envs, ≥ 5 trials — show improvement in bottleneck/branching envs
3. **Prediction RMSE**: error vs. horizon for each prediction method (killer figure at T-junctions)
4. **Ablations**: horizon K, discount γ, map patch size, residual regularization λ

in future if we can scale this work, we can even elaborate to cross embodiment in planning and navigation1 - - 对，这个延伸方向是成立的。
你现在的设计里"same model same weights for both robots"这个决定其实已经埋下了伏笔。如果未来团队里有Go2、轮式机器人、无人机混合编队，你只需要把embodiment信息编码进输入——比如加一个robot type embedding或者物理参数向量（质量、轮距、最大速度、转弯半径等），residual network就能学到不同embodiment的动力学差异。
框架的其他部分——receding-horizon rollout、joint map simulation、frontier evaluation——这些都是embodiment-agnostic的，不需要改。
但是，现在不要碰这个

## Risk
If environments are simple enough that myopic works well, horizon gains are marginal. Need bottlenecks + dead-ends + branching corridors to show the gap.
