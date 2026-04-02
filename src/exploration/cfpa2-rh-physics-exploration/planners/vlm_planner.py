from __future__ import annotations

import copy
import os
from typing import Any

from core.artifact_manager import ArtifactManager
from core.map_serializer import build_map_state_json, render_snapshot_png_b64
from core.vlm_interface import BaseVLMClient, VLMResponse, build_vlm_client

from .base_planner import BasePlanner
from .single_robot_frontier_planner import SingleRobotFrontierPlanner
from core.types import PlannerInput, PlannerOutput


class VLMPlanner(BasePlanner):
    """VLM-guided single-robot frontier planner.

    At each replan event the planner:
      1. Serializes the map state to JSON + renders a snapshot PNG.
      2. Queries the VLM: 'which frontier leads toward <goal>?'
      3. Injects a utility boost on the VLM's preferred frontier.
      4. Delegates to SingleRobotFrontierPlanner for actual path planning.
    """

    name = "vlm_frontier"

    # Class-level reference so the 3D runner can inject the image provider
    # after the simulator creates the planner internally.
    _active_instance: "VLMPlanner | None" = None

    def __init__(self, cfg: dict) -> None:
        VLMPlanner._active_instance = self
        vlm_cfg = cfg.get("vlm", {})
        self._goal_prompt: str = str(vlm_cfg.get("goal_prompt", "explore the map"))
        self._utility_boost: float = float(vlm_cfg.get("utility_boost", 5.0))

        # Build VLM client
        backend = str(vlm_cfg.get("backend", "grok"))
        api_key = str(
            vlm_cfg.get("api_key", "")
            or os.environ.get("XAI_API_KEY", "")
            or os.environ.get("GROQ_API_KEY", "")
            or os.environ.get("GEMINI_API_KEY", "")
            or os.environ.get("ANTHROPIC_API_KEY", "")
        )
        if not api_key:
            raise ValueError(
                "No VLM API key found. Set XAI_API_KEY, GROQ_API_KEY, GEMINI_API_KEY, "
                "or ANTHROPIC_API_KEY, or pass vlm.api_key in config."
            )
        self._vlm: BaseVLMClient = build_vlm_client(backend, api_key)

        # Inner planner for actual frontier selection + pathfinding
        self._inner = SingleRobotFrontierPlanner()

        # Artifact manager (injected by simulator)
        self._artifact_mgr: ArtifactManager | None = None

        # External image provider (for 3D 1st-person camera)
        self._external_image_provider: Any = None
        self._image_mode: str = "topdown"  # "topdown", "first_person", "both"

        # Cache: avoid re-querying VLM if frontiers haven't changed
        self._last_frontier_count: int = -1
        self._last_robot_target: tuple[int, int] | None = None
        self._cached_response: VLMResponse | None = None

    # ------------------------------------------------------------------
    # Injection
    # ------------------------------------------------------------------

    def set_artifact_manager(self, mgr: ArtifactManager) -> None:
        self._artifact_mgr = mgr

    def set_goal_prompt(self, goal: str) -> None:
        self._goal_prompt = goal

    def set_image_provider(self, provider: Any) -> None:
        """Inject a callable() -> str that returns a base64 PNG for the VLM.

        When set, this replaces the default top-down map snapshot.
        Use ``set_image_mode("both")`` to send *both* images to the VLM.
        """
        self._external_image_provider = provider

    def set_image_mode(self, mode: str) -> None:
        """Set VLM image mode: 'topdown' (default), 'first_person', or 'both'."""
        self._image_mode = mode

    # ------------------------------------------------------------------
    # Image stitching
    # ------------------------------------------------------------------

    @staticmethod
    def _stitch_images(left_b64: str, right_b64: str) -> str:
        """Stitch two base64 PNGs side-by-side into one image."""
        import base64
        import io
        from PIL import Image

        left = Image.open(io.BytesIO(base64.b64decode(left_b64)))
        right = Image.open(io.BytesIO(base64.b64decode(right_b64)))
        # Resize to same height
        h = max(left.height, right.height)
        if left.height != h:
            left = left.resize((int(left.width * h / left.height), h))
        if right.height != h:
            right = right.resize((int(right.width * h / right.height), h))
        combined = Image.new("RGB", (left.width + right.width, h))
        combined.paste(left, (0, 0))
        combined.paste(right, (left.width, 0))
        buf = io.BytesIO()
        combined.save(buf, format="PNG")
        buf.seek(0)
        return base64.b64encode(buf.read()).decode("ascii")

    # ------------------------------------------------------------------
    # Cache invalidation
    # ------------------------------------------------------------------

    def _should_query_vlm(self, planner_input: PlannerInput) -> bool:
        fc_count = len(planner_input.frontier_candidates)
        robot = planner_input.robot_states[0] if planner_input.robot_states else None
        robot_target = tuple(robot.current_target) if robot and robot.current_target else None

        changed = (
            self._cached_response is None
            or fc_count != self._last_frontier_count
            or robot_target != self._last_robot_target
        )
        self._last_frontier_count = fc_count
        self._last_robot_target = robot_target  # type: ignore[assignment]
        return changed

    # ------------------------------------------------------------------
    # Plan
    # ------------------------------------------------------------------

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        cfg = planner_input.config
        candidates = planner_input.frontier_candidates

        # 1. Decide whether to call VLM
        vlm_response: VLMResponse | None = self._cached_response
        if self._should_query_vlm(planner_input) and candidates:
            try:
                map_state = build_map_state_json(
                    map_mgr=planner_input.shared_map,
                    robots=planner_input.robot_states,
                    frontier_candidates=candidates,
                    step_idx=planner_input.step_idx,
                    goal_prompt=self._goal_prompt,
                    artifact_mgr=self._artifact_mgr,
                )
                # Build image(s) based on mode
                topdown_b64 = render_snapshot_png_b64(
                    map_mgr=planner_input.shared_map,
                    robots=planner_input.robot_states,
                    frontier_candidates=candidates,
                    artifact_mgr=self._artifact_mgr,
                )
                fp_b64 = None
                if self._external_image_provider is not None:
                    try:
                        fp_b64 = self._external_image_provider()
                    except Exception as img_err:
                        print(f"[VLM] 1st-person image capture failed: {img_err}")

                if self._image_mode == "first_person" and fp_b64:
                    image_b64 = fp_b64
                elif self._image_mode == "both" and fp_b64:
                    # Stitch side-by-side: 1st person left, top-down right
                    image_b64 = self._stitch_images(fp_b64, topdown_b64)
                else:
                    image_b64 = topdown_b64

                vlm_response = self._vlm.query(
                    goal_prompt=self._goal_prompt,
                    map_state=map_state,
                    image_b64=image_b64,
                )
                self._cached_response = vlm_response
                print(
                    f"[VLM] step={planner_input.step_idx} → "
                    f"frontier=({vlm_response.frontier_rep[0]},{vlm_response.frontier_rep[1]}) "
                    f"| {vlm_response.reasoning}"
                )
            except Exception as e:
                print(f"[VLM] query failed: {e}")
                vlm_response = self._cached_response

        # 2. Inject boost into config (deep copy to avoid polluting global cfg)
        boosted_cfg = copy.deepcopy(cfg)
        solo_cfg = boosted_cfg.setdefault("planning", {}).setdefault("single_robot_frontier", {})
        if vlm_response is not None:
            solo_cfg["_vlm_boost_rep"] = list(vlm_response.frontier_rep)
            solo_cfg["_vlm_boost_amount"] = self._utility_boost
        else:
            solo_cfg.pop("_vlm_boost_rep", None)
            solo_cfg.pop("_vlm_boost_amount", None)

        # 3. Delegate to inner planner
        inner_input = PlannerInput(
            shared_map=planner_input.shared_map,
            robot_states=planner_input.robot_states,
            frontier_candidates=planner_input.frontier_candidates,
            current_assignments=planner_input.current_assignments,
            reservation_state=planner_input.reservation_state,
            step_idx=planner_input.step_idx,
            sim_time=planner_input.sim_time,
            config=boosted_cfg,
        )
        output = self._inner.plan(inner_input)

        # 4. Annotate debug with VLM info
        output.planner_name = self.name
        if vlm_response is not None:
            output.debug["vlm_reasoning"] = vlm_response.reasoning
            output.debug["vlm_frontier_rep"] = list(vlm_response.frontier_rep)

        return output
