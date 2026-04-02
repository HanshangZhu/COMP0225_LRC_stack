from __future__ import annotations

import math
import time

from core.types import Pose2D, PredictorInput, PredictorOutput

from .base_predictor import BasePredictor


class ConstantVelocityPredictor(BasePredictor):
    name = "constant_velocity"

    def __init__(self, default_speed_cells_per_step: float = 0.8):
        self.default_speed = float(default_speed_cells_per_step)

    def predict(self, pred_input: PredictorInput) -> PredictorOutput:
        t0 = time.perf_counter()

        x = float(pred_input.robot_state.pose[0])
        y = float(pred_input.robot_state.pose[1])
        heading = float(pred_input.robot_state.heading_deg)
        speed = self.default_speed

        out: list[Pose2D] = []
        step_dist = speed * float(pred_input.step_dt)

        if pred_input.goal is not None:
            gx, gy = pred_input.goal
            if gx != x or gy != y:
                heading = math.degrees(math.atan2(gy - y, gx - x))

        for _ in range(max(0, int(pred_input.horizon_steps))):
            x += step_dist * math.cos(math.radians(heading))
            y += step_dist * math.sin(math.radians(heading))
            out.append(Pose2D(x=x, y=y, heading_deg=heading, speed=speed))

        t1 = time.perf_counter()
        return PredictorOutput(trajectory=out, inference_time_sec=t1 - t0, debug={"model": self.name})
