#!/usr/bin/env python3
"""Fail-closed deployment revision and ID0 yaw preflight helpers."""

from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess

from parkingbot_ops import (
    ROLES, local_ros_argv, remote_run, role_host, role_workspace,
    workspace_revision_command,
)

ID0_YAW_HELPER = Path(__file__).resolve().with_name("id0_yaw_preflight.py")
SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")

ID0_YAW_DURATION_S = 5.0
ID0_YAW_MIN_SAMPLES = 30
ID0_YAW_MIN_VISIBLE_RATIO = 0.80
ID0_YAW_MAX_STD_DEG = 2.0
ID0_YAW_MAX_DEVIATION_DEG = 5.0
ID0_YAW_MAX_STEP_DEG = 5.0


def evaluate_revisions(controller_head: str, remote_heads: dict[str, str]):
    errors = []
    if not SHA_PATTERN.fullmatch(controller_head or ""):
        errors.append("controller: git HEAD unavailable")
    for role in ROLES:
        value = remote_heads.get(role, "")
        if not SHA_PATTERN.fullmatch(value or ""):
            errors.append(f"{role}: git HEAD unavailable")
        elif controller_head and value != controller_head:
            errors.append(
                f"{role}: code revision differs "
                f"(controller={controller_head}, remote={value})")
    return errors


def local_workspace_revision(workspace) -> str:
    root = Path(workspace)
    for repository in (
            root, root / "src/cooperative_parking_robot",
            root / "ros2/cooperative_parking_robot"):
        result = subprocess.run(
            ["git", "-C", str(repository), "rev-parse", "HEAD"],
            text=True, capture_output=True)
        if result.returncode == 0 and SHA_PATTERN.fullmatch(
                result.stdout.strip()):
            return result.stdout.strip()
    marker = root / ".parkingbot_revision"
    if marker.is_file():
        return marker.read_text(encoding="utf-8").splitlines()[0].strip()
    return ""


def revision_status(config, runner, *, controller_head: str | None = None):
    if controller_head is None:
        controller_head = local_workspace_revision(config["CONTROL_WORKSPACE"])

    remote_heads = {}
    for role in ROLES:
        result = remote_run(
            runner, role_host(config, role),
            workspace_revision_command(role_workspace(config, role)), timeout=7)
        remote_heads[role] = (
            result.stdout.strip() if result.returncode == 0 else "")

    errors = evaluate_revisions(controller_head, remote_heads)
    return {
        "ok": not errors,
        "controller_head": controller_head,
        "remote_heads": remote_heads,
        "errors": errors,
    }


def _last_json_object(text: str):
    for line in reversed(str(text).splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    return None


def run_id0_yaw_preflight(config, runner):
    argv = local_ros_argv(config, [
        "python3", str(ID0_YAW_HELPER),
        "--duration", str(ID0_YAW_DURATION_S),
        "--min-samples", str(ID0_YAW_MIN_SAMPLES),
        "--min-visible-ratio", str(ID0_YAW_MIN_VISIBLE_RATIO),
        "--max-std-deg", str(ID0_YAW_MAX_STD_DEG),
        "--max-deviation-deg", str(ID0_YAW_MAX_DEVIATION_DEG),
        "--max-step-deg", str(ID0_YAW_MAX_STEP_DEG),
    ])
    result = runner.run(argv, timeout=ID0_YAW_DURATION_S + 5.0)
    payload = _last_json_object(result.stdout)
    if payload is None:
        payload = {
            "passed": False,
            "reason": "ID0_YAW_PREFLIGHT_NO_RESULT",
            "returncode": result.returncode,
            "stderr": str(result.stderr).strip(),
        }
    else:
        payload["returncode"] = result.returncode
        if result.returncode != 0:
            payload["passed"] = False
    return payload
