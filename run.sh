#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Conda's ROS libraries are ABI-incompatible with the system ROS installation.
# Preserve desktop access and ROS networking choices, but rebuild the runtime
# environment without inherited Python, library, or workspace search paths.
runtime_env=("HOME=$HOME" "PATH=/usr/bin:/bin:/usr/sbin:/sbin" "PYTHONNOUSERSITE=1")
# All Gazebo processes run locally. Avoid discovery selecting a VPN interface.
runtime_env+=("GZ_IP=${GZ_IP:-127.0.0.1}")
for name in USER LOGNAME LANG LC_ALL DISPLAY WAYLAND_DISPLAY XAUTHORITY \
    XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS ROS_DOMAIN_ID \
    ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; do
    if [[ -v $name ]]; then
        runtime_env+=("$name=${!name}")
    fi
done

exec /usr/bin/env -i "${runtime_env[@]}" /bin/bash --noprofile --norc -s -- "$workspace" "$@" <<'BASH'
set -eo pipefail
cd -- "$1"
shift
source /opt/ros/jazzy/setup.bash
# Clear cached CMake paths from earlier builds made with Conda activated.
colcon build --symlink-install --cmake-clean-cache
source install/local_setup.bash
exec ros2 launch my_robot_description gazebo.launch.py "$@"
BASH
