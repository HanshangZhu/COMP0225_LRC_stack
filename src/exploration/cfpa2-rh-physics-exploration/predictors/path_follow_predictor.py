from __future__ import annotations

import math
import time

from core.types import Pose2D, PredictorInput, PredictorOutput

from .base_predictor import BasePredictor


class PathFollowPredictor(BasePredictor):
    name = "path_follow"

    def predict(self, pred_input: PredictorInput) -> PredictorOutput:
        t0 = time.perf_counter()

        x = float(pred_input.robot_state.pose[0])
        y = float(pred_input.robot_state.pose[1])
        heading = float(pred_input.robot_state.heading_deg)
        max_speed = float(pred_input.local_context.get("max_speed_cells_per_step", 1.0))

        path = list(pred_input.current_path)
        out: list[Pose2D] = []

        idx = 0
        for _ in range(max(0, int(pred_input.horizon_steps))):
            if idx < len(path):
                nx, ny = path[idx]
                dx = float(nx) - x
                dy = float(ny) - y
                if dx != 0.0 or dy != 0.0:
                    heading = math.degrees(math.atan2(dy, dx))
                x = float(nx)
                y = float(ny)
                idx += 1
            elif pred_input.goal is not None:
                gx, gy = pred_input.goal
                dx = float(gx) - x
                dy = float(gy) - y
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    pass
                else:
                    heading = math.degrees(math.atan2(dy, dx))
                    dist = math.hypot(dx, dy)
                    step = min(max_speed * pred_input.step_dt, dist)
                    if dist > 1e-6:
                        x += step * (dx / dist)
                        y += step * (dy / dist)
            out.append(Pose2D(x=x, y=y, heading_deg=heading, speed=max_speed))

        t1 = time.perf_counter()
        return PredictorOutput(
            trajectory=out,
            inference_time_sec=t1 - t0,
            debug={"model": self.name, "used_path_prefix": int(min(len(path), pred_input.horizon_steps))},
        )
