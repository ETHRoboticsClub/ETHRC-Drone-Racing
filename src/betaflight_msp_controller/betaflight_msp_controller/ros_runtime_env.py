import os


def _running_under_wsl():
    if os.environ.get("WSL_INTEROP") or os.environ.get("WSL_DISTRO_NAME"):
        return True

    try:
        with open("/proc/sys/kernel/osrelease", "r", encoding="utf-8") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


def configure_local_ros_defaults():
    """Keep local WSL SITL tools on one DDS discovery island by default."""
    if os.environ.get("ETHRC_DISABLE_LOCAL_ROS_DEFAULTS") == "1":
        return

    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    if not _running_under_wsl():
        return

    if os.environ.get("ETHRC_ALLOW_SUBNET_DISCOVERY") == "1":
        return

    if os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE"):
        return

    discovery = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    if discovery in ("", "SUBNET"):
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
