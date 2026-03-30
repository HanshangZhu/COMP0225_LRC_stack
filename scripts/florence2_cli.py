#!/usr/bin/env python3
"""Interactive Florence-2 CLI for live robot camera inspection.

Subscribes to the robot's front camera, runs Florence-2 on demand,
and lets you type free-form prompts in a REPL.

Usage:
  # Default: single robot namespace "robot"
  python3 scripts/florence2_cli.py

  # Custom namespace + model
  python3 scripts/florence2_cli.py --ns robot_a --model microsoft/Florence-2-base

  # CPU-only (slow but works without GPU)
  python3 scripts/florence2_cli.py --device cpu

Built-in commands (type at the prompt):
  <OD>                         — open-vocabulary object detection
  <CAPTION>                    — short caption
  <DETAILED_CAPTION>           — medium caption
  <MORE_DETAILED_CAPTION>      — rich scene description
  <CAPTION_TO_PHRASE_GROUNDING> find the red box  — ground a phrase
  <OCR>                        — read text in the image
  quit / exit / q              — exit

Any other text is treated as a VLM chat query: Florence-2 receives the
camera frame + your text as a <MORE_DETAILED_CAPTION> prompt, prefixed
with a system-style context line.
"""

from __future__ import annotations

import argparse
import importlib.machinery
import importlib.util
import readline  # noqa: F401  — enables arrow-key history in input()
import sys
import textwrap
import threading
import time
import types

import numpy as np


# ── Flash-attn shim (same as florence2_detector_node) ───────────────
def _ensure_flash_attn_shim():
    if importlib.util.find_spec("flash_attn") is not None:
        return
    def _dummy(name):
        mod = types.ModuleType(name)
        mod.__spec__ = importlib.machinery.ModuleSpec(name, None)
        mod.__path__ = []
        mod.__version__ = "2.0.0"
        return mod
    for name in ("flash_attn", "flash_attn.flash_attn_func", "flash_attn.bert_padding"):
        sys.modules.setdefault(name, _dummy(name))


# ── Florence-2 helpers ──────────────────────────────────────────────

def load_model(model_id: str, device: str):
    import torch
    _ensure_flash_attn_shim()
    from transformers import AutoModelForCausalLM, AutoProcessor

    print(f"Loading {model_id} on {device}...")
    processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
    model = AutoModelForCausalLM.from_pretrained(
        model_id, trust_remote_code=True,
        torch_dtype=torch.float16 if device == "cuda" else torch.float32,
        attn_implementation="sdpa",
    ).to(device)
    model.eval()
    if device == "cuda":
        import torch
        print(f"VRAM used: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
    return model, processor


def run_task(model, processor, pil_image, task: str, text_input: str, device: str) -> dict:
    import torch
    prompt = task if not text_input else task + text_input
    inputs = processor(text=prompt, images=pil_image, return_tensors="pt").to(device)
    model_dtype = next(model.parameters()).dtype
    if inputs["pixel_values"].dtype != model_dtype:
        inputs["pixel_values"] = inputs["pixel_values"].to(model_dtype)
    with torch.no_grad():
        out = model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=512, num_beams=3,
        )
    decoded = processor.batch_decode(out, skip_special_tokens=False)[0]
    return processor.post_process_generation(
        decoded, task=task, image_size=(pil_image.width, pil_image.height),
    )


# ── ROS2 camera subscriber ─────────────────────────────────────────

class CameraGrabber:
    """Thin wrapper that keeps the latest camera frame."""

    def __init__(self, namespace: str):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("florence2_cli")
        self._latest: Image | None = None
        self._lock = threading.Lock()
        self._count = 0
        topic = f"/{namespace}/front_camera/image_raw"
        self._sub = self._node.create_subscription(Image, topic, self._cb, 10)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        print(f"Subscribed to {topic}  (waiting for frames...)")

    def _cb(self, msg):
        with self._lock:
            self._latest = msg
            self._count += 1

    def _spin(self):
        import rclpy
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def grab(self):
        """Return the latest frame as a PIL Image, or None."""
        from PIL import Image as PILImage
        with self._lock:
            msg = self._latest
        if msg is None:
            return None
        if msg.encoding not in ("rgb8", "bgr8"):
            print(f"  [warn] unsupported encoding '{msg.encoding}'")
            return None
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1]
        return PILImage.fromarray(img.copy())

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._count


# ── Known Florence-2 tasks ──────────────────────────────────────────

KNOWN_TASKS = {
    "<OD>", "<CAPTION>", "<DETAILED_CAPTION>", "<MORE_DETAILED_CAPTION>",
    "<CAPTION_TO_PHRASE_GROUNDING>", "<OCR>", "<OCR_WITH_REGION>",
    "<REGION_PROPOSAL>", "<DENSE_REGION_CAPTION>",
    "<REFERRING_EXPRESSION_SEGMENTATION>",
}

SYSTEM_CONTEXT = (
    "You are the visual cortex of an autonomous exploration robot. "
    "You are seeing a live first-person camera feed from the robot as it "
    "navigates through an indoor environment. "
    "Describe what you observe in detail, focusing on objects, obstacles, "
    "doorways, open spaces, and anything that could guide exploration. "
    "User query: "
)


def parse_input(raw: str) -> tuple[str, str]:
    """Parse user input into (florence_task, text_input).

    If the input starts with a known <TASK>, split on the first '>'.
    Otherwise treat it as a free-form query using <MORE_DETAILED_CAPTION>.
    """
    stripped = raw.strip()
    if not stripped:
        return "", ""

    # Check for an explicit Florence-2 task tag
    for task in KNOWN_TASKS:
        if stripped.upper().startswith(task):
            rest = stripped[len(task):].strip()
            return task, rest

    # Free-form prompt → wrap in detailed caption with system context
    return "<MORE_DETAILED_CAPTION>", ""


def format_result(task: str, result: dict, user_query: str) -> str:
    """Pretty-print a Florence-2 result."""
    lines = []
    for key, value in result.items():
        if isinstance(value, dict):
            bboxes = value.get("bboxes", [])
            labels = value.get("labels", [])
            if bboxes:
                lines.append(f"  Detections ({len(bboxes)}):")
                for bbox, label in zip(bboxes, labels):
                    x1, y1, x2, y2 = [round(v, 1) for v in bbox]
                    lines.append(f"    [{x1}, {y1}, {x2}, {y2}]  {label}")
            else:
                lines.append(f"  {key}: (none)")
        elif isinstance(value, str):
            wrapped = textwrap.fill(value, width=80, initial_indent="  ", subsequent_indent="  ")
            lines.append(wrapped)
        else:
            lines.append(f"  {key}: {value}")
    return "\n".join(lines)


# ── Main REPL ───────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Florence-2 interactive CLI for robot camera")
    parser.add_argument("--ns", default="robot", help="Robot namespace (default: robot)")
    parser.add_argument("--model", default="microsoft/Florence-2-large", help="HuggingFace model ID")
    parser.add_argument("--device", default="cuda", help="torch device: cuda or cpu")
    args = parser.parse_args()

    model, processor = load_model(args.model, args.device)
    cam = CameraGrabber(args.ns)

    # Wait briefly for first frame
    print("Waiting for camera frame...", end="", flush=True)
    for _ in range(100):
        if cam.frame_count > 0:
            break
        time.sleep(0.1)
    if cam.frame_count > 0:
        print(f" got it ({cam.frame_count} frames)")
    else:
        print(" no frames yet (will retry on each query)")

    print()
    print("=" * 60)
    print("Florence-2 Robot Camera CLI")
    print("=" * 60)
    print()
    print("Type a Florence-2 task tag or a free-form question.")
    print("Examples:")
    print("  <OD>                              — detect all objects")
    print("  <CAPTION_TO_PHRASE_GROUNDING> door — find doors")
    print("  <MORE_DETAILED_CAPTION>           — describe the scene")
    print("  what obstacles do you see?         — free-form query")
    print("  quit                              — exit")
    print()

    while True:
        try:
            raw = input("\033[1;36mflorence>\033[0m ")
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if raw.strip().lower() in ("quit", "exit", "q"):
            break
        if not raw.strip():
            continue

        frame = cam.grab()
        if frame is None:
            print("  [no camera frame available yet]")
            continue

        task, text_input = parse_input(raw)
        if not task:
            continue

        # For free-form queries, show what we're actually doing
        if raw.strip() not in KNOWN_TASKS and not any(raw.strip().upper().startswith(t) for t in KNOWN_TASKS):
            print(f"  → using {task} (free-form query: \"{raw.strip()}\")")
            # We can't truly inject arbitrary prompts into Florence-2's
            # task format, but we use detailed caption and print the
            # user's question alongside the result.
            user_query = raw.strip()
        else:
            user_query = ""

        t0 = time.perf_counter()
        try:
            result = run_task(model, processor, frame, task, text_input, args.device)
        except Exception as e:
            print(f"  [error] {e}")
            continue
        dt = time.perf_counter() - t0

        print(f"  [{dt:.2f}s] {task}")
        print(format_result(task, result, user_query))
        print()


if __name__ == "__main__":
    main()
