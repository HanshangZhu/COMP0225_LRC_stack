"""Pipeline config loader/validator for Isaac exploration launches.

The schema is intentionally lightweight:
- launch validates envelope/registry fields
- individual modules own validation of their params
"""

from __future__ import annotations

import copy
import json
import os
from typing import Any


DEFAULT_MAPPING_PARAMS = {
    "max_scan_odom_dt": 0.10,
    "max_map_odom_dt": 0.50,
    "odom_history_sec": 2.0,
    "mapper_update_rate": 2.0,
    "frontier_update_rate": 2.0,
}

DEFAULT_REACTIVE_NAV_PARAMS = {
    "max_linear_speed": 0.28,
    "obstacle_slow_dist": 0.85,
    "obstacle_stop_dist": 0.40,
    "planner_grid_radius": 6.0,
    "planner_goal_clip_distance": 4.0,
    "wall_scan_enabled": True,
}


def _merge_missing(dst: dict[str, Any], src: dict[str, Any]) -> None:
    for key, value in src.items():
        if key not in dst:
            dst[key] = copy.deepcopy(value)
            continue
        if isinstance(dst[key], dict) and isinstance(value, dict):
            _merge_missing(dst[key], value)


def _load_ros_params_yaml(path: str) -> dict[str, Any]:
    if not path:
        return {}
    if not os.path.exists(path):
        return {}

    def _parse_scalar(raw: str) -> Any:
        text = raw.strip()
        if not text:
            return ""
        if text.startswith("\"") and text.endswith("\"") and len(text) >= 2:
            return text[1:-1]
        if text.startswith("'") and text.endswith("'") and len(text) >= 2:
            return text[1:-1]

        lowered = text.lower()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
        if lowered == "null":
            return None

        try:
            if any(ch in text for ch in (".", "e", "E")):
                return float(text)
            return int(text)
        except ValueError:
            return text

    params: dict[str, Any] = {}
    in_ros_params = False
    params_indent = -1

    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].rstrip("\n")
            if not line.strip():
                continue

            indent = len(line) - len(line.lstrip(" "))
            stripped = line.strip()

            if stripped == "ros__parameters:":
                in_ros_params = True
                params_indent = indent
                continue

            if not in_ros_params:
                continue

            if indent <= params_indent:
                break

            if ":" not in stripped:
                continue

            key, value = stripped.split(":", 1)
            key = key.strip()
            if not key:
                continue
            params[key] = _parse_scalar(value)

    return params


def _validate_modules(modules: dict[str, Any], key: str, require_node: bool) -> list[str]:
    errors: list[str] = []
    entries = modules.get(key)
    if not isinstance(entries, list) or not entries:
        errors.append(f"modules.{key} must be a non-empty list")
        return errors

    seen_ids: set[str] = set()
    for idx, entry in enumerate(entries):
        tag = f"modules.{key}[{idx}]"
        if not isinstance(entry, dict):
            errors.append(f"{tag} must be an object")
            continue

        module_id = entry.get("id")
        if not isinstance(module_id, str) or not module_id:
            errors.append(f"{tag}.id must be a non-empty string")
        elif module_id in seen_ids:
            errors.append(f"duplicate module id '{module_id}' in modules.{key}")
        else:
            seen_ids.add(module_id)

        has_passthrough_mode = entry.get("mode") == "passthrough"
        if require_node and not has_passthrough_mode:
            node = entry.get("node")
            if not isinstance(node, dict):
                errors.append(f"{tag}.node must be an object")
                continue
            package = node.get("package")
            executable = node.get("executable")
            if not isinstance(package, str) or not package:
                errors.append(f"{tag}.node.package must be a non-empty string")
            if not isinstance(executable, str) or not executable:
                errors.append(f"{tag}.node.executable must be a non-empty string")
        elif not require_node:
            if not has_passthrough_mode:
                node = entry.get("node")
                if not isinstance(node, dict):
                    errors.append(
                        f"{tag} must define either mode='passthrough' or node.package/node.executable"
                    )
                    continue
                package = node.get("package")
                executable = node.get("executable")
                if not isinstance(package, str) or not package:
                    errors.append(f"{tag}.node.package must be a non-empty string")
                if not isinstance(executable, str) or not executable:
                    errors.append(f"{tag}.node.executable must be a non-empty string")

    return errors


def validate_pipeline_config(config: dict[str, Any]) -> None:
    errors: list[str] = []

    version = config.get("version")
    if not isinstance(version, str):
        errors.append("version must be a string")
    elif not version.startswith("1."):
        errors.append(f"unsupported schema version '{version}' (expected 1.x)")

    defaults = config.get("defaults")
    if not isinstance(defaults, dict):
        errors.append("defaults must be an object")

    modules = config.get("modules")
    if not isinstance(modules, dict):
        errors.append("modules must be an object")
    else:
        errors.extend(_validate_modules(modules, "frontier_planners", require_node=True))
        errors.extend(_validate_modules(modules, "goal_assigners", require_node=True))
        errors.extend(_validate_modules(modules, "global_planners", require_node=False))

    if errors:
        raise ValueError("Invalid pipeline config:\n- " + "\n- ".join(errors))


def _index_modules(config: dict[str, Any], key: str) -> dict[str, dict[str, Any]]:
    indexed: dict[str, dict[str, Any]] = {}
    for entry in config["modules"][key]:
        indexed[str(entry["id"])] = entry
    return indexed


def _resolve_module(
    module_map: dict[str, dict[str, Any]],
    selected_id: str,
    module_name: str,
) -> dict[str, Any]:
    if selected_id in module_map:
        return module_map[selected_id]
    valid = ", ".join(sorted(module_map.keys()))
    raise ValueError(f"Unknown {module_name} '{selected_id}'. Valid IDs: {valid}")


def _clean_override_dict(overrides: dict[str, Any] | None) -> dict[str, Any]:
    if not overrides:
        return {}
    out: dict[str, Any] = {}
    for key, value in overrides.items():
        if value is None:
            continue
        out[key] = value
    return out


def _merge_overrides(base: dict[str, Any], overrides: dict[str, Any] | None) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for key, value in _clean_override_dict(overrides).items():
        out[key] = value
    return out


def load_pipeline_runtime(
    *,
    pipeline_config_json: str,
    frontier_planner_id: str,
    goal_assigner_id: str,
    global_planner_id: str,
    use_legacy_yaml_fallback: bool,
    legacy_frontier_yaml: str,
    legacy_reactive_nav_yaml: str,
    mapping_overrides: dict[str, Any] | None = None,
    reactive_nav_overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not pipeline_config_json:
        raise ValueError("pipeline_config_json must be set")
    if not os.path.exists(pipeline_config_json):
        raise FileNotFoundError(f"pipeline_config_json not found: {pipeline_config_json}")

    with open(pipeline_config_json, "r", encoding="utf-8") as f:
        raw = json.load(f)

    if not isinstance(raw, dict):
        raise ValueError("pipeline config root must be an object")

    validate_pipeline_config(raw)
    config = copy.deepcopy(raw)

    mapping = config.setdefault("mapping", {})
    if not isinstance(mapping, dict):
        raise ValueError("mapping must be an object")
    mapping_params = mapping.setdefault("params", {})
    if not isinstance(mapping_params, dict):
        raise ValueError("mapping.params must be an object")

    navigation = config.setdefault("navigation", {})
    if not isinstance(navigation, dict):
        raise ValueError("navigation must be an object")
    reactive_nav = navigation.setdefault("reactive_nav", {})
    if not isinstance(reactive_nav, dict):
        raise ValueError("navigation.reactive_nav must be an object")
    reactive_nav_params = reactive_nav.setdefault("params", {})
    if not isinstance(reactive_nav_params, dict):
        raise ValueError("navigation.reactive_nav.params must be an object")

    _merge_missing(mapping_params, DEFAULT_MAPPING_PARAMS)
    _merge_missing(reactive_nav_params, DEFAULT_REACTIVE_NAV_PARAMS)

    fallback_sources: list[str] = []
    if use_legacy_yaml_fallback:
        legacy_frontier_params = _load_ros_params_yaml(legacy_frontier_yaml)
        legacy_reactive_nav_params = _load_ros_params_yaml(legacy_reactive_nav_yaml)

        if legacy_frontier_params:
            fallback_sources.append(legacy_frontier_yaml)
            _merge_missing(mapping_params, {
                "frontier_update_rate": legacy_frontier_params.get("update_rate"),
            })
            for frontier_mod in config["modules"]["frontier_planners"]:
                params = frontier_mod.setdefault("params", {})
                if isinstance(params, dict):
                    _merge_missing(params, legacy_frontier_params)
        if legacy_reactive_nav_params:
            fallback_sources.append(legacy_reactive_nav_yaml)
            _merge_missing(reactive_nav_params, legacy_reactive_nav_params)

    defaults = config["defaults"]
    selected_frontier_id = frontier_planner_id or str(defaults.get("frontier_planner_id", ""))
    selected_goal_assigner_id = goal_assigner_id or str(defaults.get("goal_assigner_id", ""))
    selected_global_planner_id = global_planner_id or str(defaults.get("global_planner_id", ""))

    if not selected_frontier_id:
        raise ValueError("No frontier planner selected (defaults.frontier_planner_id missing)")
    if not selected_goal_assigner_id:
        raise ValueError("No goal assigner selected (defaults.goal_assigner_id missing)")
    if not selected_global_planner_id:
        raise ValueError("No global planner selected (defaults.global_planner_id missing)")

    frontier_map = _index_modules(config, "frontier_planners")
    goal_assigner_map = _index_modules(config, "goal_assigners")
    global_planner_map = _index_modules(config, "global_planners")

    frontier_module = _resolve_module(frontier_map, selected_frontier_id, "frontier_planner_id")
    goal_assigner_module = _resolve_module(goal_assigner_map, selected_goal_assigner_id, "goal_assigner_id")
    global_planner_module = _resolve_module(global_planner_map, selected_global_planner_id, "global_planner_id")

    resolved_mapping_params = _merge_overrides(mapping_params, mapping_overrides)
    resolved_reactive_nav_params = _merge_overrides(reactive_nav_params, reactive_nav_overrides)

    return {
        "raw": config,
        "selected_ids": {
            "frontier_planner_id": selected_frontier_id,
            "goal_assigner_id": selected_goal_assigner_id,
            "global_planner_id": selected_global_planner_id,
        },
        "frontier_module": frontier_module,
        "goal_assigner_module": goal_assigner_module,
        "global_planner_module": global_planner_module,
        "mapping_params": resolved_mapping_params,
        "reactive_nav_params": resolved_reactive_nav_params,
        "fallback_sources": fallback_sources,
        "valid_ids": {
            "frontier_planner_ids": sorted(frontier_map.keys()),
            "goal_assigner_ids": sorted(goal_assigner_map.keys()),
            "global_planner_ids": sorted(global_planner_map.keys()),
        },
    }
