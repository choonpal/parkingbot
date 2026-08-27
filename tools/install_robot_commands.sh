#!/usr/bin/env bash
set -euo pipefail

tool_dir="$(cd "$(dirname "$0")" && pwd)"
install_dir="${HOME}/.local/bin"
config_dir="${HOME}/.config/parkingbot"
mkdir -p "$install_dir"
mkdir -p "$config_dir"

# The public robotctl command is the fail-closed guard. The previous CLI is
# retained as robotctl_core so its mature start/state/log/stop implementation
# remains unchanged behind the revision gate.
install -m 0755 "$tool_dir/robotctl" "$install_dir/robotctl"
install -m 0755 "$tool_dir/robotctl_core" "$install_dir/robotctl_core"
install -m 0644 "$tool_dir/parkingbot_ops.py" "$install_dir/parkingbot_ops.py"
install -m 0644 "$tool_dir/production_preflight.py" \
  "$install_dir/production_preflight.py"
install -m 0755 "$tool_dir/id0_yaw_preflight.py" \
  "$install_dir/id0_yaw_preflight.py"
install -m 0755 "$tool_dir/parkingbot_ros_snapshot.py" \
  "$install_dir/parkingbot_ros_snapshot.py"
for command in start state logs stop restart doctor id0_check; do
  ln -sfn robotctl "$install_dir/robot_${command}"
done
if [[ ! -f "$config_dir/production_hosts.env" ]]; then
  install -m 0600 "$tool_dir/production_hosts.env.example" \
    "$config_dir/production_hosts.env"
fi

echo "Installed guarded robotctl and robot_* commands in $install_dir"
case ":${PATH}:" in
  *":${install_dir}:"*) ;;
  *)
    echo "PATH does not include $install_dir"
    echo "For this shell run: export PATH=\"${install_dir}:\${PATH}\""
    echo "Add it to your shell configuration manually if desired."
    ;;
esac
echo "Required site configuration: $config_dir/production_hosts.env"
echo "Fill it with verified host, workspace, device and measured geometry values."
echo "Before cooperative motion, align the robots and run: robot_id0_check"
