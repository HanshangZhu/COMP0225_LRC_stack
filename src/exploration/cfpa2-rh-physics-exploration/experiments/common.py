from __future__ import annotations

import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

from core.config import write_json


def make_run_id(prefix: str = "run") -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{ts}"


def git_commit_hash() -> str:
    try:
        return (
            subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL)
            .decode("utf-8")
            .strip()
        )
    except Exception:
        return "unknown"


def prepare_output_dirs(output_root: str | Path, run_id: str) -> dict[str, Path]:
    root = Path(output_root) / "benchmarks" / run_id
    dirs = {
        "root": root,
        "configs": root / "configs",
        "metadata": root / "metadata",
        "episode": root / "episode_logs",
        "results_csv": root / "results_csv",
        "plots": root / "plots",
        "animations": root / "animations",
    }
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)
    return dirs


def enforce_mp4_only(cfg: dict[str, Any]) -> dict[str, Any]:
    experiment_cfg = dict(cfg.get("experiment", {}))
    animation_cfg = dict(cfg.get("animation", {}))
    experiment_cfg["animation_format"] = "mp4"
    animation_cfg["save_gif"] = False
    animation_cfg["save_mp4"] = True
    cfg["experiment"] = experiment_cfg
    cfg["animation"] = animation_cfg
    return cfg


def save_run_metadata(path: str | Path, payload: dict[str, Any]) -> None:
    write_json(path, payload)
