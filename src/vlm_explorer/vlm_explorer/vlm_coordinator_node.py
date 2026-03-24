#!/usr/bin/env python3
"""VLM Coordinator: centralized VLM-in-the-loop waypoint assignment.

Consumes rendered map image + scene JSON, constructs a hybrid prompt,
queries a VLM (OpenAI GPT-4o or Anthropic Claude), parses the response, and publishes
waypoint assignments for each robot.

Placeholder agentic tool: print_green_reached — when a robot is within
reach_radius of a detected green marker, the VLM can invoke this tool.

Falls back to the existing CFPA2 coordinator when VLM is disabled or fails.
"""

from __future__ import annotations

import base64
import json
import math
import os
import tempfile
import time
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ── VLM Tool Definitions ────────────────────────────────────────────────
VLM_TOOLS = [
    {
        "name": "assign_waypoints",
        "description": (
            "Assign exploration waypoints to robots. Each robot gets a target "
            "(x, y) in world coordinates. Choose frontiers that maximize "
            "exploration coverage while keeping robots apart."
        ),
        "parameters": {
            "assignments": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "robot": {"type": "string", "description": "Robot namespace, e.g. robot_a"},
                        "x": {"type": "number", "description": "Target X in world frame"},
                        "y": {"type": "number", "description": "Target Y in world frame"},
                        "reason": {"type": "string", "description": "Brief reason for this assignment"},
                    },
                },
            }
        },
    },
    {
        "name": "print_green_reached",
        "description": (
            "Call this when a robot is close to a detected green marker. "
            "This signals that the robot has successfully reached a green "
            "target object. Provide the robot name and marker ID."
        ),
        "parameters": {
            "robot": {"type": "string", "description": "Robot that reached the marker"},
            "marker_id": {"type": "string", "description": "ID of the green marker reached"},
            "message": {"type": "string", "description": "Optional message about the discovery"},
        },
    },
]

SYSTEM_PROMPT = """\
You are a multi-robot exploration coordinator. You see an annotated occupancy grid map \
showing explored (white), unexplored (gray), and obstacle (dark) regions.

Robot positions are shown as colored circles with heading lines:
  - Green circle = robot_a
  - Blue circle = robot_b

Yellow dots mark the topological skeleton (corridor structure).
Bright green circles mark detected green target objects.

Your job:
1. Assign each robot a waypoint that maximizes exploration coverage.
2. Keep robots spread apart — avoid sending both to the same area.
3. Prefer unexplored (gray) regions adjacent to explored (white) areas (frontiers).
4. If a robot is near a green marker, call print_green_reached.
5. Respond with ONLY a JSON object with tool calls. No extra text.

Response format:
{
  "tool_calls": [
    {"name": "assign_waypoints", "arguments": {"assignments": [...]}},
    {"name": "print_green_reached", "arguments": {"robot": "...", "marker_id": "...", "message": "..."}}
  ]
}
"""


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VLMCoordinatorNode(Node):
    def __init__(self):
        super().__init__("vlm_coordinator")

        self.declare_parameter("robot_namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("rendered_map_topic", "/vlm/rendered_map")
        self.declare_parameter("scene_json_topic", "/vlm/scene_json")
        self.declare_parameter("green_detections_topic", "/vlm/green_detections")
        self.declare_parameter("goal_topic_suffix", "/way_point_coord")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("replan_period_sec", 20.0)
        self.declare_parameter("vlm_enabled", True)
        self.declare_parameter("vlm_provider", "openai")  # "openai" or "anthropic"
        self.declare_parameter("vlm_model", "gpt-4o")
        self.declare_parameter("vlm_temperature", 0.2)
        self.declare_parameter("vlm_max_tokens", 1024)
        self.declare_parameter("vlm_max_retries", 3)
        self.declare_parameter("green_reach_radius_m", 1.0)
        # Fallback: if VLM fails, hold last known waypoint
        self.declare_parameter("fallback_hold_sec", 40.0)

        self._namespaces = list(self.get_parameter("robot_namespaces").value)
        self._goal_suffix = self.get_parameter("goal_topic_suffix").value
        self._frame_id = self.get_parameter("frame_id").value
        self._replan_sec = self.get_parameter("replan_period_sec").value
        self._vlm_enabled = self.get_parameter("vlm_enabled").value
        self._vlm_provider = self.get_parameter("vlm_provider").value
        self._vlm_model = self.get_parameter("vlm_model").value
        self._vlm_temp = self.get_parameter("vlm_temperature").value
        self._vlm_max_tokens = self.get_parameter("vlm_max_tokens").value
        self._vlm_retries = self.get_parameter("vlm_max_retries").value
        self._reach_radius = self.get_parameter("green_reach_radius_m").value

        # Subscriptions
        self._rendered_img = None
        self._scene_json = None
        self._green_dets = []
        self._odoms = {}

        self.create_subscription(
            Image,
            self.get_parameter("rendered_map_topic").value,
            self._on_rendered,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("scene_json_topic").value,
            self._on_scene_json,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("green_detections_topic").value,
            self._on_green_dets,
            10,
        )
        for ns in self._namespaces:
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav",
                lambda msg, _ns=ns: self._on_odom(_ns, msg), 10,
            )

        # Publishers: one waypoint publisher per robot
        self._goal_pubs = {}
        for ns in self._namespaces:
            topic = f"/{ns}{self._goal_suffix}"
            self._goal_pubs[ns] = self.create_publisher(PointStamped, topic, 10)

        # State
        self._last_goals: dict[str, tuple[float, float]] = {}
        self._last_plan_time = 0.0
        self._green_reached: set[str] = set()

        self._timer = self.create_timer(self._replan_sec, self._replan_tick)

        # Also publish at 2 Hz to keep reactive_nav fed with the last goal
        self._goal_repeat_timer = self.create_timer(0.5, self._republish_goals)

        self.get_logger().info(
            f"VLMCoordinator: vlm={self._vlm_enabled} provider={self._vlm_provider} "
            f"model={self._vlm_model} T_H={self._replan_sec}s ns={self._namespaces}"
        )

    def _on_rendered(self, msg: Image):
        self._rendered_img = msg

    def _on_scene_json(self, msg: String):
        self._scene_json = msg.data

    def _on_green_dets(self, msg: String):
        try:
            self._green_dets = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    # ── Replan tick ──────────────────────────────────────────────────
    def _replan_tick(self):
        """Called every T_H seconds."""
        if self._rendered_img is None or self._scene_json is None:
            self.get_logger().info("VLM replan: waiting for rendered map + scene JSON")
            return

        # Check green marker proximity → auto-trigger print_green_reached
        self._check_green_proximity()

        if not self._vlm_enabled:
            self.get_logger().debug("VLM disabled; relying on CFPA2 fallback.")
            return

        # Build VLM query
        scene = {}
        try:
            scene = json.loads(self._scene_json)
        except (json.JSONDecodeError, TypeError):
            pass

        scene["green_markers"] = self._green_dets
        scene["green_already_reached"] = list(self._green_reached)
        scene["tools_available"] = [t["name"] for t in VLM_TOOLS]

        # Encode rendered image as base64 PNG-ish (raw RGB, but APIs want base64 JPEG/PNG)
        img_b64 = self._encode_image_b64()

        if img_b64 is None:
            self.get_logger().warn("VLM replan: could not encode map image")
            return

        # Query VLM
        response = self._query_vlm(img_b64, scene)

        if response is None:
            self.get_logger().warn("VLM query failed; keeping last goals.")
            return

        # Parse and execute tool calls
        self._execute_tool_calls(response)

    def _check_green_proximity(self):
        """Auto-trigger print_green_reached if robot is near a green marker."""
        for det in self._green_dets:
            mid = det.get("id", "")
            if mid in self._green_reached:
                continue
            mx, my = det.get("x", 0), det.get("y", 0)
            for ns in self._namespaces:
                if ns not in self._odoms:
                    continue
                rx = self._odoms[ns].pose.pose.position.x
                ry = self._odoms[ns].pose.pose.position.y
                dist = math.sqrt((rx - mx) ** 2 + (ry - my) ** 2)
                if dist < self._reach_radius:
                    self._green_reached.add(mid)
                    self.get_logger().info(
                        f"*** print_green_reached: {ns} reached {mid} "
                        f"at ({mx:.1f}, {my:.1f}) dist={dist:.2f}m ***"
                    )

    def _encode_image_b64(self) -> Optional[str]:
        """Encode the latest rendered map image as base64 PNG."""
        msg = self._rendered_img
        if msg is None:
            return None
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
            # Encode as PNG using raw Python (no cv2 dependency)
            # Use a simple PPM→base64 encoding that VLM APIs can accept
            # Actually, let's write a minimal PNG or just use raw bytes description
            # For API compatibility, produce a JPEG via PIL if available, else PPM
            try:
                from PIL import Image as PILImage
                import io
                pil_img = PILImage.fromarray(img)
                buf = io.BytesIO()
                pil_img.save(buf, format="JPEG", quality=80)
                return base64.b64encode(buf.getvalue()).decode("ascii")
            except ImportError:
                # Fallback: encode raw PPM
                header = f"P6\n{msg.width} {msg.height}\n255\n".encode()
                ppm_bytes = header + img.tobytes()
                return base64.b64encode(ppm_bytes).decode("ascii")
        except (ValueError, IndexError):
            return None

    def _query_vlm(self, img_b64: str, scene: dict) -> Optional[dict]:
        """Query VLM (OpenAI or Anthropic) and return parsed JSON response."""
        user_content = (
            f"Current scene data:\n```json\n{json.dumps(scene, indent=2)}\n```\n\n"
            "Based on the map image and scene data above, assign waypoints to each robot "
            "and call print_green_reached if any robot is near a green marker. "
            "Respond with ONLY JSON."
        )

        if self._vlm_provider == "anthropic":
            api_key = os.environ.get("ANTHROPIC_API_KEY", "")
            env_name = "ANTHROPIC_API_KEY"
            call_fn = self._call_anthropic_vision
        else:
            api_key = os.environ.get("OPENAI_API_KEY", "")
            env_name = "OPENAI_API_KEY"
            call_fn = self._call_openai_vision

        if not api_key:
            self.get_logger().warn(f"{env_name} not set; using dummy planner.")
            return self._dummy_vlm_response()

        for attempt in range(self._vlm_retries):
            try:
                result = call_fn(api_key, img_b64, user_content)
                if result is not None:
                    return result
            except Exception as e:
                self.get_logger().warn(
                    f"VLM attempt {attempt+1}/{self._vlm_retries} failed: {e}"
                )

        return None

    def _call_openai_vision(self, api_key: str, img_b64: str, user_content: str) -> Optional[dict]:
        """Call OpenAI GPT vision API (gpt-4o, gpt-4o-mini, gpt-4-turbo, etc.)."""
        import urllib.request

        payload = {
            "model": self._vlm_model,
            "temperature": self._vlm_temp,
            "max_tokens": self._vlm_max_tokens,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{img_b64}",
                                "detail": "low",
                            },
                        },
                        {"type": "text", "text": user_content},
                    ],
                },
            ],
        }

        req = urllib.request.Request(
            "https://api.openai.com/v1/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {api_key}",
            },
        )
        with urllib.request.urlopen(req, timeout=15) as resp:
            body = json.loads(resp.read().decode("utf-8"))

        text = body["choices"][0]["message"]["content"]
        self.get_logger().info(f"VLM response ({self._vlm_provider}/{self._vlm_model}): {text[:200]}")
        return self._parse_vlm_json(text)

    def _call_anthropic_vision(self, api_key: str, img_b64: str, user_content: str) -> Optional[dict]:
        """Call Anthropic Messages API (claude-sonnet-4-20250514, etc.)."""
        import urllib.request

        payload = {
            "model": self._vlm_model,
            "max_tokens": self._vlm_max_tokens,
            "system": SYSTEM_PROMPT,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": "image/jpeg",
                                "data": img_b64,
                            },
                        },
                        {"type": "text", "text": user_content},
                    ],
                },
            ],
        }
        if self._vlm_temp is not None:
            payload["temperature"] = self._vlm_temp

        req = urllib.request.Request(
            "https://api.anthropic.com/v1/messages",
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "x-api-key": api_key,
                "anthropic-version": "2023-06-01",
            },
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            body = json.loads(resp.read().decode("utf-8"))

        text = body["content"][0]["text"]
        self.get_logger().info(f"VLM response ({self._vlm_provider}/{self._vlm_model}): {text[:200]}")
        return self._parse_vlm_json(text)

    def _dummy_vlm_response(self) -> Optional[dict]:
        """Dummy response when no API key is set — spreads robots to frontiers.

        Uses scene JSON to find green markers and assigns robots toward them,
        or falls back to simple directional spreading.
        """
        self.get_logger().info("VLM: no API key, using dummy planner")

        assignments = []
        targets = list(self._green_dets)

        for i, ns in enumerate(self._namespaces):
            if ns not in self._odoms:
                continue
            rx = self._odoms[ns].pose.pose.position.x
            ry = self._odoms[ns].pose.pose.position.y

            if i < len(targets):
                # Send robot toward a green marker
                t = targets[i]
                tx, ty = t["x"], t["y"]
            else:
                # Default: spread robots in different directions
                angle = (i * math.pi / len(self._namespaces))
                tx = rx + 3.0 * math.cos(angle)
                ty = ry + 3.0 * math.sin(angle)

            assignments.append({
                "robot": ns,
                "x": round(tx, 2),
                "y": round(ty, 2),
                "reason": "dummy_vlm_planner",
            })

        return {
            "tool_calls": [
                {"name": "assign_waypoints", "arguments": {"assignments": assignments}}
            ]
        }

    def _parse_vlm_json(self, text: str) -> Optional[dict]:
        """Parse VLM response text as JSON, stripping markdown fences."""
        text = text.strip()
        if text.startswith("```"):
            lines = text.split("\n")
            text = "\n".join(lines[1:])
            if text.endswith("```"):
                text = text[:-3]
        try:
            return json.loads(text.strip())
        except json.JSONDecodeError:
            self.get_logger().warn(f"VLM response not valid JSON: {text[:200]}")
            return None

    # ── Execute tool calls ───────────────────────────────────────────
    def _execute_tool_calls(self, response: dict):
        tool_calls = response.get("tool_calls", [])
        if not tool_calls:
            self.get_logger().warn("VLM response had no tool_calls")
            return

        for tc in tool_calls:
            name = tc.get("name", "")
            args = tc.get("arguments", {})

            if name == "assign_waypoints":
                self._handle_assign_waypoints(args)
            elif name == "print_green_reached":
                self._handle_print_green_reached(args)
            else:
                self.get_logger().warn(f"Unknown VLM tool: {name}")

    def _handle_assign_waypoints(self, args: dict):
        assignments = args.get("assignments", [])
        for a in assignments:
            ns = a.get("robot", "")
            x = float(a.get("x", 0))
            y = float(a.get("y", 0))
            reason = a.get("reason", "")

            if ns not in self._goal_pubs:
                self.get_logger().warn(f"VLM assigned to unknown robot: {ns}")
                continue

            self._last_goals[ns] = (x, y)
            self._publish_goal(ns, x, y)
            self.get_logger().info(
                f"VLM → {ns}: ({x:.2f}, {y:.2f}) reason={reason}"
            )

    def _handle_print_green_reached(self, args: dict):
        robot = args.get("robot", "unknown")
        marker_id = args.get("marker_id", "unknown")
        message = args.get("message", "")
        self._green_reached.add(marker_id)
        self.get_logger().info(
            f"*** TOOL: print_green_reached ***\n"
            f"  Robot: {robot}\n"
            f"  Marker: {marker_id}\n"
            f"  Message: {message}\n"
            f"  All reached: {self._green_reached}"
        )

    def _publish_goal(self, ns: str, x: float, y: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self._goal_pubs[ns].publish(msg)

    def _republish_goals(self):
        """Re-publish last known goals at 2 Hz to keep reactive_nav alive."""
        for ns, (x, y) in self._last_goals.items():
            self._publish_goal(ns, x, y)


def main(args=None):
    rclpy.init(args=args)
    node = VLMCoordinatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
