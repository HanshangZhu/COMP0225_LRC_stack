#!/usr/bin/env python3
"""Deterministic copy-first generator for ROS2 exact TARE tree.

The generator intentionally performs mechanical, reproducible transforms only.
It does not attempt algorithm rewrites.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def sorted_file_list(root: Path) -> list[Path]:
    files = []
    for current_root, dirnames, filenames in os.walk(root):
        dirnames.sort()
        filenames.sort()
        for name in filenames:
            files.append(Path(current_root) / name)
    return files


def copy_tree(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst, symlinks=False)


def mechanical_rewrite(path: Path) -> None:
    # Keep transforms intentionally conservative and deterministic.
    if path.suffix not in {".h", ".hpp", ".hh", ".c", ".cc", ".cpp", ".py", ".sh", ".txt", ".cmake", ".xml"}:
        return
    try:
        raw = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return
    rewritten = raw
    rewritten = rewritten.replace("#include <ros/ros.h>", "#include <rclcpp/rclcpp.hpp>")
    rewritten = rewritten.replace("#include \"ros/ros.h\"", "#include <rclcpp/rclcpp.hpp>")
    rewritten = rewritten.replace("ROS_INFO(", "RCLCPP_INFO(rclcpp::get_logger(\"tare_ros2_exact\"), ")
    rewritten = rewritten.replace("ROS_WARN(", "RCLCPP_WARN(rclcpp::get_logger(\"tare_ros2_exact\"), ")
    rewritten = rewritten.replace("ROS_ERROR(", "RCLCPP_ERROR(rclcpp::get_logger(\"tare_ros2_exact\"), ")
    if rewritten != raw:
        path.write_text(rewritten, encoding="utf-8")


def collect_manifest(root: Path) -> dict[str, str]:
    files = sorted_file_list(root)
    manifest: dict[str, str] = {}
    for file_path in files:
        rel = file_path.relative_to(root).as_posix()
        manifest[rel] = sha256_file(file_path)
    return manifest


def try_git_commit(path: Path) -> str | None:
    try:
        out = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        return out or None
    except (subprocess.SubprocessError, FileNotFoundError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        default="src/mtare_ros1_ws/src/mtare_planner/src/tare_planner",
        help="ROS1 TARE source tree to mirror into upstream/tare_planner.",
    )
    parser.add_argument(
        "--refresh-upstream",
        action="store_true",
        help="Refresh upstream/tare_planner from --source-root before generation.",
    )
    args = parser.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    workspace_root = package_root.parents[1]
    source_root = (workspace_root / args.source_root).resolve()
    upstream_root = package_root / "upstream" / "tare_planner"
    generated_root = package_root / "generated" / "tare_planner"
    manifest_path = package_root / "UPSTREAM_MANIFEST.json"

    if args.refresh_upstream:
        if not source_root.exists():
            raise FileNotFoundError(
                f"--refresh-upstream requested but source tree is missing: {source_root}"
            )
        copy_tree(source_root, upstream_root)

    if not upstream_root.exists():
        raise FileNotFoundError(
            f"Upstream mirror missing: {upstream_root}. Run with --refresh-upstream first."
        )

    copy_tree(upstream_root, generated_root)
    for file_path in sorted_file_list(generated_root):
        mechanical_rewrite(file_path)

    upstream_manifest = collect_manifest(upstream_root)
    manifest_payload = {
        "source_root": args.source_root,
        "source_git_commit": try_git_commit(workspace_root),
        "upstream_root": "upstream/tare_planner",
        "file_count": len(upstream_manifest),
        "files": upstream_manifest,
    }
    manifest_path.write_text(json.dumps(manifest_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"Generated ROS2 tree at: {generated_root}")
    print(f"Updated manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
