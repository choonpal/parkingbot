from __future__ import annotations

from pathlib import Path
import shlex


DEFAULT_SITE_CONFIG = Path(
    "~/.config/parkingbot/production_hosts.env").expanduser()


def load_site_config(path: str | Path) -> dict[str, str]:
    """Load the existing ParkingBot KEY=VALUE site configuration.

    Manual site launchers intentionally reuse the same file as robotctl so
    measured device paths and geometry have one source of truth.
    """
    target = Path(path).expanduser()
    if not target.is_file():
        raise RuntimeError(
            f"ParkingBot site config not found: {target}. "
            "Copy tools/production_hosts.env.example to "
            "~/.config/parkingbot/production_hosts.env and fill site values.")

    values: dict[str, str] = {}
    for number, raw in enumerate(target.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise RuntimeError(
                f"{target}:{number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        key = key.strip()
        parsed = shlex.split(value, comments=True)
        values[key] = parsed[0] if parsed else ""

    values.setdefault("ROS_DOMAIN_ID", "42")
    values.setdefault("ROS_LOCALHOST_ONLY", "0")
    values.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    values.setdefault("REAR_CAMERA_TOPIC", "/rear/marker_camera/image")
    values.setdefault("REAR_ENABLE_INTERNAL_CAMERA", "true")
    return values


def require_site_keys(
        values: dict[str, str], keys: tuple[str, ...], role: str) -> None:
    missing = [key for key in keys if not values.get(key, "").strip()]
    if missing:
        joined = ", ".join(missing)
        raise RuntimeError(
            f"{role} site config is incomplete; set: {joined}")


def site_bool(values: dict[str, str], key: str, default: bool) -> bool:
    raw = values.get(key, "true" if default else "false").strip().lower()
    if raw in ("1", "true", "yes", "on"):
        return True
    if raw in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"{key} must be true or false, got {raw!r}")
