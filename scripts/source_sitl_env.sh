#!/usr/bin/env bash
# Source this file from the repository root before running SITL commands:
#   source scripts/source_sitl_env.sh

_ethrc_sitl_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_ethrc_detect_ros_distro() {
    if [ -n "${ROS_DISTRO:-}" ] && [ -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        printf '%s\n' "$ROS_DISTRO"
        return
    fi

    local distro
    for distro in jazzy rolling humble iron; do
        if [ -r "/opt/ros/${distro}/setup.bash" ]; then
            printf '%s\n' "$distro"
            return
        fi
    done

    return 1
}

_ethrc_detect_wsl_ipv4() {
    local ip_addr

    if command -v ip >/dev/null 2>&1; then
        ip_addr="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{
            for (i = 1; i <= NF; i++) {
                if ($i == "src") {
                    print $(i + 1)
                    exit
                }
            }
        }')"
        if [ -n "$ip_addr" ]; then
            printf '%s\n' "$ip_addr"
            return 0
        fi
    fi

    if command -v hostname >/dev/null 2>&1; then
        hostname -I 2>/dev/null | awk '{print $1}'
        return 0
    fi

    return 1
}

_ethrc_write_fastdds_wsl_profile() {
    local ip_addr="$1"
    local profile_path
    local profile_dir

    if [ -z "$ip_addr" ]; then
        return 1
    fi

    profile_path="${ETHRC_FASTDDS_WSL_PROFILE:-${XDG_RUNTIME_DIR:-/tmp}/ethrc_fastdds_wsl_${UID:-$(id -u)}.xml}"
    profile_dir="$(dirname "$profile_path")"
    mkdir -p "$profile_dir" 2>/dev/null || return 1

    cat >"$profile_path" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>ethrc_wsl_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>${ip_addr}</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="ros2_default_profile" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>ethrc_wsl_udp</transport_id>
      </userTransports>
    </rtps>
  </participant>
</profiles>
EOF

    printf '%s\n' "$profile_path"
}

_ethrc_ros_distro="$(_ethrc_detect_ros_distro)" || {
    echo "[source_sitl_env.sh] No ROS 2 setup.bash found under /opt/ros." >&2
    return 1 2>/dev/null || exit 1
}

# shellcheck disable=SC1090
source "/opt/ros/${_ethrc_ros_distro}/setup.bash"

if [ -r "${_ethrc_sitl_env_dir}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${_ethrc_sitl_env_dir}/install/setup.bash"
else
    echo "[source_sitl_env.sh] Workspace install/setup.bash not found; run colcon build first." >&2
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null || [ -n "${WSL_INTEROP:-}" ]; then
    if [ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ] && [ "${ETHRC_DISABLE_WSL_FASTDDS_PROFILE:-0}" != "1" ] && [ -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]; then
        _ethrc_wsl_ipv4="$(_ethrc_detect_wsl_ipv4)"
        _ethrc_fastdds_profile="$(_ethrc_write_fastdds_wsl_profile "$_ethrc_wsl_ipv4")"
        if [ -n "$_ethrc_fastdds_profile" ]; then
            export FASTDDS_DEFAULT_PROFILES_FILE="$_ethrc_fastdds_profile"
            export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
            if [ -z "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" ] || [ "${ROS_AUTOMATIC_DISCOVERY_RANGE}" = "LOCALHOST" ]; then
                export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
            fi
        else
            export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
        fi
    else
        export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
    fi
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export GZ_IP="${GZ_IP:-127.0.0.1}"
fi

export GZ_SIM_RESOURCE_PATH="${_ethrc_sitl_env_dir}/sim/gazebo/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${_ethrc_sitl_env_dir}/install/sitl_gazebo_plugins/lib:${_ethrc_sitl_env_dir}/ext/aeroloop_gazebo/plugins/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export ETHRC_BOREAS_CONFIG="${ETHRC_BOREAS_CONFIG:-${_ethrc_sitl_env_dir}/config/airframes/boreas_4s.yaml}"

echo "[source_sitl_env.sh] ROS_DISTRO=${_ethrc_ros_distro} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset}"
if [ -n "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "[source_sitl_env.sh] FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE}"
fi

unset _ethrc_sitl_env_dir
unset _ethrc_ros_distro
unset _ethrc_wsl_ipv4
unset _ethrc_fastdds_profile
unset -f _ethrc_detect_ros_distro
unset -f _ethrc_detect_wsl_ipv4
unset -f _ethrc_write_fastdds_wsl_profile
