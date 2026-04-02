from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

Cell = tuple[int, int]


@dataclass
class Pose2D:
    x: float
    y: float
    heading_deg: float
    speed: float = 0.0


@dataclass
class FrontierCandidate:
    representative: Cell
    cells: list[Cell]

    @property
    def size(self) -> int:
        return len(self.cells)


@dataclass
class GoalAssignment:
    robot_id: int
    target: Cell | None
    path: list[Cell]
    utility: float
    valid: bool
    breakdown: dict[str, float] = field(default_factory=dict)


@dataclass
class RobotState:
    robot_id: int
    pose: Cell
    heading_deg: float
    velocity: tuple[float, float] = (0.0, 0.0)
    current_target: Cell | None = None
    path: list[Cell] = field(default_factory=list)
    trajectory: list[Cell] = field(default_factory=list)
    steps_since_progress: int = 0
    idle_steps: int = 0
    total_steps: int = 0
    total_move_steps: int = 0
    revisited_move_steps: int = 0
    path_length: float = 0.0

    def __post_init__(self) -> None:
        self.trajectory.append(self.pose)

    def set_plan(self, target: Cell | None, path: list[Cell] | None) -> None:
        self.current_target = target
        self.steps_since_progress = 0
        if not path:
            self.path = []
            return
        if path and path[0] == self.pose:
            self.path = list(path[1:])
        else:
            self.path = list(path)

    def at_target(self) -> bool:
        return self.current_target is not None and self.pose == self.current_target

    @property
    def speed(self) -> float:
        vx, vy = self.velocity
        return (vx * vx + vy * vy) ** 0.5


@dataclass
class PlannerInput:
    shared_map: Any
    robot_states: list[RobotState]
    frontier_candidates: list[FrontierCandidate]
    current_assignments: dict[int, GoalAssignment]
    reservation_state: dict[Cell, dict[str, int]]
    step_idx: int
    sim_time: float
    config: dict[str, Any]


@dataclass
class PlannerOutput:
    planner_name: str
    assignments: dict[int, GoalAssignment]
    joint_score: float
    score_breakdown: dict[str, float] = field(default_factory=dict)
    predicted_paths: dict[int, list[Cell]] = field(default_factory=dict)
    debug: dict[str, Any] = field(default_factory=dict)


@dataclass
class PredictorInput:
    robot_state: RobotState
    goal: Cell | None
    current_path: list[Cell]
    local_context: dict[str, Any]
    horizon_steps: int
    step_dt: float


@dataclass
class PredictorOutput:
    trajectory: list[Pose2D]
    inference_time_sec: float
    uncertainty: list[float] = field(default_factory=list)
    debug: dict[str, Any] = field(default_factory=dict)
