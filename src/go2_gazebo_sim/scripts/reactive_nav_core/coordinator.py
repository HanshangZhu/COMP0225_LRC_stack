from __future__ import annotations

import math

from .controller import MotionController
from .geometry import wrap_angle
from .perception import ScanAnalyzer
from .planner import LocalPlanner
from .recovery import RecoveryManager
from .state import TickResult


class ReactiveNavCoordinator:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.scan_analyzer = ScanAnalyzer()
        self.planner = LocalPlanner(cfg)
        self.recovery = RecoveryManager(cfg)
        self.controller = MotionController(cfg)

    @staticmethod
    def _reset_plan_cache(runtime_state) -> None:
        runtime_state.plan_waypoints_world = []
        runtime_state.plan_last_time_sec = None
        runtime_state.plan_last_goal = None

    def _apply_action_common(
        self,
        action,
        now_sec: float,
        runtime_state,
        robot_state,
        scan,
        goal_dx_world: float,
        goal_dy_world: float,
        events: list[tuple[str, str]],
    ) -> bool:
        events.extend(action.events)
        if action.request_replan:
            self._reset_plan_cache(runtime_state)
        if action.request_escape_reason is not None:
            ok, escape_events = self.planner.set_temporary_escape_target(
                now_sec,
                runtime_state,
                robot_state,
                scan,
                goal_dx_world,
                goal_dy_world,
                reason=action.request_escape_reason,
            )
            events.extend(escape_events)
            if not ok:
                events.append(("warn", "No viable scan-based escape target found."))
        return action.consume_tick

    def tick(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        goal_state,
        scan,
        external_stop: int,
    ) -> TickResult:
        events: list[tuple[str, str]] = []

        if runtime_state.start_time_sec is None:
            runtime_state.start_time_sec = now_sec
        if (now_sec - runtime_state.start_time_sec) < self.cfg.startup_delay:
            return TickResult(0.0, 0.0, events=events, diagnostics={"mode": "startup"})

        if self.cfg.require_settle_before_motion and not runtime_state.settle_ready:
            if robot_state.speed > self.cfg.settle_speed_threshold:
                runtime_state.settle_start_time_sec = None
                return TickResult(0.0, 0.0, events=events, diagnostics={"mode": "settling"})

            if runtime_state.settle_start_time_sec is None:
                runtime_state.settle_start_time_sec = now_sec
                return TickResult(0.0, 0.0, events=events, diagnostics={"mode": "settling"})

            settle_elapsed = now_sec - runtime_state.settle_start_time_sec
            if settle_elapsed < self.cfg.settle_hold_sec:
                return TickResult(0.0, 0.0, events=events, diagnostics={"mode": "settling"})

            runtime_state.settle_ready = True
            events.append(
                (
                    "info",
                    f"Settle gate passed (speed<{self.cfg.settle_speed_threshold:.2f} m/s "
                    f"for {self.cfg.settle_hold_sec:.1f}s).",
                )
            )

        if goal_state.x is None or goal_state.y is None:
            return TickResult(0.0, 0.0, events=events, diagnostics={"mode": "no_goal"})

        if scan is None:
            return TickResult(0.0, 0.0, events=events, diagnostics={"mode": "no_scan"})

        goal_dx_world = goal_state.x - robot_state.x
        goal_dy_world = goal_state.y - robot_state.y
        dist_to_goal = math.hypot(goal_dx_world, goal_dy_world)
        if dist_to_goal < self.cfg.goal_tolerance:
            return TickResult(0.0, 0.0, events=events, diagnostics={
                "mode": "goal_reached",
                "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                "dist_goal": round(dist_to_goal, 2),
            })

        goal_angle = math.atan2(goal_dy_world, goal_dx_world)
        heading_err_goal = wrap_angle(goal_angle - robot_state.yaw)

        scan_metrics = self.scan_analyzer.analyze(scan, self.cfg)
        blocked_sec = self.recovery.update_blocked_state(
            now_sec,
            runtime_state,
            robot_state,
            scan_metrics.min_front,
            external_stop,
        )
        self.recovery.update_scan_rearm(runtime_state, robot_state, scan_metrics.min_front)

        trigger_action = self.recovery.maybe_trigger_scan(
            now_sec,
            runtime_state,
            robot_state,
            scan_metrics.min_front,
            blocked_sec,
            heading_err_goal,
            dist_to_goal,
            external_stop,
        )
        self._apply_action_common(
            trigger_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        )

        scan_action = self.recovery.step_scan(now_sec, runtime_state, robot_state)
        if self._apply_action_common(
            scan_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        ):
            mode = "wall_scan" if scan_action.mode == "scan_turn" else "scan_pause"
            return TickResult(
                scan_action.linear_x,
                scan_action.angular_z,
                request_replan=scan_action.request_replan,
                events=events,
                diagnostics={
                    "mode": mode,
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "dist_goal": round(dist_to_goal, 2),
                    "min_front": round(scan_metrics.min_front, 2),
                    "blocked_sec": round(blocked_sec, 1),
                    "ext_stop": external_stop,
                },
            )

        unstick_action = self.recovery.step_unstick(now_sec, runtime_state)
        if self._apply_action_common(
            unstick_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        ):
            return TickResult(
                unstick_action.linear_x,
                unstick_action.angular_z,
                request_replan=unstick_action.request_replan,
                events=events,
                diagnostics={
                    "mode": "unstick",
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "dist_goal": round(dist_to_goal, 2),
                    "min_front": round(scan_metrics.min_front, 2),
                    "blocked_sec": round(blocked_sec, 1),
                    "ext_stop": external_stop,
                },
            )

        blocked_action = self.recovery.maybe_trigger_blocked_recovery(
            now_sec,
            runtime_state,
            robot_state,
            scan_metrics.left_push,
            scan_metrics.right_push,
            heading_err_goal,
            dist_to_goal,
            blocked_sec,
            scan_metrics.rear_clearance,
        )
        self._apply_action_common(
            blocked_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        )

        self.planner.clear_stale_escape_target(now_sec, robot_state, runtime_state)

        target_x = goal_state.x
        target_y = goal_state.y
        steering_source = "goal"
        if runtime_state.escape_target_world is not None:
            target_x, target_y = runtime_state.escape_target_world
            steering_source = "escape"
        elif self.cfg.planner_enabled:
            plan_result = self.planner.planner_target_world(
                now_sec,
                runtime_state,
                robot_state,
                goal_state,
                scan,
                goal_dx_world,
                goal_dy_world,
            )
            events.extend(plan_result.events)
            if plan_result.target_world is not None:
                target_x, target_y = plan_result.target_world
                steering_source = "planner"

        target_dx = target_x - robot_state.x
        target_dy = target_y - robot_state.y
        target_angle = math.atan2(target_dy, target_dx)
        heading_err = wrap_angle(target_angle - robot_state.yaw)

        lin, ang = self.controller.compute_cmd(
            heading_err,
            heading_err_goal,
            scan_metrics.min_front,
            scan_metrics.left_push,
            scan_metrics.right_push,
            external_stop,
        )

        has_plan = len(runtime_state.plan_waypoints_world) > 0
        plan_wps = len(runtime_state.plan_waypoints_world)
        escape = None
        if runtime_state.escape_target_world is not None:
            escape = [round(runtime_state.escape_target_world[0], 2),
                      round(runtime_state.escape_target_world[1], 2)]

        diag = {
            "mode": "navigate",
            "steer": steering_source,
            "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
            "dist_goal": round(dist_to_goal, 2),
            "target": [round(target_x, 2), round(target_y, 2)],
            "min_front": round(scan_metrics.min_front, 2),
            "blocked_sec": round(blocked_sec, 1),
            "has_plan": has_plan,
            "plan_wps": plan_wps,
            "escape": escape,
            "ext_stop": external_stop,
        }

        return TickResult(
            linear_x=lin,
            angular_z=ang,
            request_replan=blocked_action.request_replan or unstick_action.request_replan,
            events=events,
            diagnostics=diag,
        )
