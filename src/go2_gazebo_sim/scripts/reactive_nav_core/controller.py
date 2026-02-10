from __future__ import annotations

import math


class MotionController:
    def __init__(self, cfg) -> None:
        self.cfg = cfg

    def compute_cmd(
        self,
        heading_err: float,
        heading_err_goal: float,
        min_front: float,
        left_push: float,
        right_push: float,
        external_stop: int,
    ) -> tuple[float, float]:
        lin = self.cfg.max_linear_speed

        heading_factor = max(0.0, math.cos(heading_err))
        lin *= heading_factor

        if min_front < self.cfg.obstacle_slow_dist:
            denom = max(1e-6, self.cfg.obstacle_slow_dist - self.cfg.obstacle_stop_dist)
            speed_scale = max(0.0, (min_front - self.cfg.obstacle_stop_dist) / denom)
            lin *= speed_scale

        if min_front < self.cfg.obstacle_stop_dist or external_stop != 0:
            lin = 0.0

        lin = max(0.0, min(lin, self.cfg.max_linear_speed))

        goal_turn = self.cfg.max_angular_speed * (2.0 / math.pi) * heading_err
        goal_turn = max(-self.cfg.max_angular_speed, min(goal_turn, self.cfg.max_angular_speed))

        avoid_raw = right_push - left_push
        if abs(avoid_raw) < self.cfg.avoidance_deadband:
            avoid_raw = 0.0
        avoid_yaw = self.cfg.avoidance_gain * avoid_raw

        if min_front < self.cfg.obstacle_stop_dist and self.cfg.turn_in_place_on_block:
            avoid_yaw = 0.0
        elif avoid_yaw * heading_err_goal < 0.0:
            avoid_yaw *= self.cfg.avoidance_conflict_scale

        max_avoid = max(0.0, self.cfg.avoidance_max_ratio) * self.cfg.max_angular_speed
        avoid_yaw = max(-max_avoid, min(avoid_yaw, max_avoid))

        ang = goal_turn + avoid_yaw
        ang = max(-self.cfg.max_angular_speed, min(ang, self.cfg.max_angular_speed))

        return (float(lin), float(ang))
