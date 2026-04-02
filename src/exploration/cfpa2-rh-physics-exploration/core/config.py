from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml


def deep_merge(base: dict[str, Any], updates: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in updates.items():
        if isinstance(out.get(key), dict) and isinstance(value, dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_yaml(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_experiment_config(
    base_cfg_path: str | Path,
    planner_cfg_path: str | Path | None = None,
    env_cfg_path: str | Path | None = None,
    extra_override: dict[str, Any] | None = None,
) -> dict[str, Any]:
    cfg = load_yaml(base_cfg_path)
    if planner_cfg_path is not None:
        cfg = deep_merge(cfg, load_yaml(planner_cfg_path))
    if env_cfg_path is not None:
        cfg = deep_merge(cfg, load_yaml(env_cfg_path))
    if extra_override:
        cfg = deep_merge(cfg, extra_override)
    return cfg


def write_config_snapshot(path: str | Path, cfg: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
