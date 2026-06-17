#!/usr/bin/env bash
set -euo pipefail

# Setup script for Betaflight SITL + Gazebo Harmonic simulation verification.

SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly SCRIPT_NAME

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly WORKSPACE_DIR

SUDO=()
APT_UPDATED=0
COLCON_PACKAGES=(
    drone_control
    betaflight_msp_controller
    gazebo_ros_bridges
    sitl_gazebo_plugins
    rl
    interfaces
)

log() {
    echo "[$SCRIPT_NAME] $*" >&2
}

die() {
    echo "[$SCRIPT_NAME] Error: $*" >&2
    exit 1
}

have_cmd() {
    command -v "$1" >/dev/null 2>&1
}

sudo_cmd() {
    if [ "$(id -u)" -eq 0 ]; then
        SUDO=()
    elif have_cmd sudo; then
        SUDO=(sudo)
    else
        die "This script needs root privileges for apt operations. Re-run as root or install sudo."
    fi
}

require_cmd() {
    have_cmd "$1" || die "Required command '$1' was not found. Install it and re-run this script."
}

apt_update_once() {
    if [ "$APT_UPDATED" -eq 0 ]; then
        "${SUDO[@]}" apt-get update || log "Warning: apt-get update failed, but proceeding..."
        APT_UPDATED=1
    fi
}

apt_package_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

apt_package_available() {
    apt-cache show "$1" >/dev/null 2>&1
}

package_source_dir() {
    local package_name="$1"
    local package_dir

    package_dir="$(find "$WORKSPACE_DIR/src" -mindepth 2 -maxdepth 4 -name package.xml -printf '%h\n' \
        | while IFS= read -r candidate; do
            if grep -Eq "<name>[[:space:]]*$package_name[[:space:]]*</name>" "$candidate/package.xml"; then
                printf '%s\n' "$candidate"
                break
            fi
        done)"

    [ -n "$package_dir" ] || return 1
    printf '%s\n' "$package_dir"
}

validate_colcon_package_sources() {
    local package_name
    local package_dir
    local missing_packages=()

    for package_name in "$@"; do
        if ! package_dir="$(package_source_dir "$package_name")"; then
            missing_packages+=("$package_name")
            continue
        fi

        [ -r "$package_dir/package.xml" ] \
            || die "Package '$package_name' is missing a readable package.xml at $package_dir."
    done

    if [ "${#missing_packages[@]}" -gt 0 ]; then
        die "Missing ROS package source directories for: ${missing_packages[*]}"
    fi
}

clean_colcon_package_artifacts() {
    local package_name="$1"

    log "Cleaning stale colcon artifacts for package '$package_name'."
    rm -rf \
        "$WORKSPACE_DIR/build/$package_name" \
        "$WORKSPACE_DIR/install/$package_name"
}

package_from_artifact_path() {
    local path="$1"
    local relative

    case "$path" in
        "$WORKSPACE_DIR/build/"*)
            relative="${path#"$WORKSPACE_DIR/build/"}"
            ;;
        "$WORKSPACE_DIR/install/"*)
            relative="${path#"$WORKSPACE_DIR/install/"}"
            ;;
        *)
            return 1
            ;;
    esac

    relative="${relative%%/*}"
    [ -n "$relative" ] || return 1
    printf '%s\n' "$relative"
}

cleanup_stale_colcon_symlinks() {
    local roots=()
    local broken_links=()
    local packages_to_clean=()
    local root
    local link_path
    local package_name
    local seen

    for root in "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install"; do
        [ -d "$root" ] && roots+=("$root")
    done

    [ "${#roots[@]}" -gt 0 ] || return

    mapfile -t broken_links < <(find "${roots[@]}" -xtype l -print 2>/dev/null || true)
    [ "${#broken_links[@]}" -gt 0 ] || return

    log "Found stale colcon symlinks from an older build:"
    for link_path in "${broken_links[@]}"; do
        log "  $link_path -> $(readlink "$link_path" || true)"
        if package_name="$(package_from_artifact_path "$link_path")"; then
            seen=" ${packages_to_clean[*]} "
            if [[ "$seen" != *" $package_name "* ]]; then
                packages_to_clean+=("$package_name")
            fi
        fi
    done

    if [ "${#packages_to_clean[@]}" -eq 0 ]; then
        log "Removing stale symlinks directly because no package owner could be inferred."
        rm -f "${broken_links[@]}"
        return
    fi

    for package_name in "${packages_to_clean[@]}"; do
        clean_colcon_package_artifacts "$package_name"
    done
}

install_apt_packages() {
    local packages=()
    local pkg

    for pkg in "$@"; do
        if ! apt_package_installed "$pkg"; then
            packages+=("$pkg")
        fi
    done

    if [ "${#packages[@]}" -eq 0 ]; then
        log "Apt packages already installed: $*"
        return
    fi

    apt_update_once
    log "Installing apt packages: ${packages[*]}"
    "${SUDO[@]}" apt-get install -y "${packages[@]}"
}

detect_os() {
    if [ ! -r /etc/os-release ]; then
        die "Cannot detect the operating system because /etc/os-release is missing."
    fi

    # shellcheck disable=SC1091
    . /etc/os-release

    OS_ID="${ID:-}"
    OS_CODENAME="${VERSION_CODENAME:-}"

    if [ -z "$OS_CODENAME" ] && have_cmd lsb_release; then
        OS_CODENAME="$(lsb_release -cs)"
    fi

    if [ "$OS_ID" != "ubuntu" ]; then
        die "Unsupported OS '$OS_ID'. This installer currently supports Ubuntu apt-based systems."
    fi

    [ -n "$OS_CODENAME" ] || die "Could not detect the Ubuntu/Debian codename."
    log "Detected OS: $OS_ID $OS_CODENAME"
}

default_ros_distro_for_os() {
    case "$OS_CODENAME" in
        noble)
            echo "jazzy"
            ;;
        jammy)
            echo "humble"
            ;;
        *)
            echo ""
            ;;
    esac
}

detect_ros_distro() {
    local distro="${ROS_DISTRO:-}"
    local default_distro

    if [ -n "$distro" ]; then
        echo "$distro"
        return
    fi

    default_distro="$(default_ros_distro_for_os)"
    if [ -n "$default_distro" ] && [ -d "/opt/ros/$default_distro" ]; then
        echo "$default_distro"
        return
    fi

    for distro in jazzy humble rolling iron; do
        if [ -d "/opt/ros/$distro" ]; then
            echo "$distro"
            return
        fi
    done

    echo "$default_distro"
}

source_ros_setup() {
    ROS_DISTRO_DETECTED="$(detect_ros_distro)"

    if [ -z "$ROS_DISTRO_DETECTED" ]; then
        die "Could not infer a ROS 2 distro for $OS_CODENAME. Source ROS first or set ROS_DISTRO."
    fi

    if [ ! -r "/opt/ros/$ROS_DISTRO_DETECTED/setup.bash" ]; then
        die "ROS 2 '$ROS_DISTRO_DETECTED' is not installed at /opt/ros/$ROS_DISTRO_DETECTED. Install ROS 2 first or set ROS_DISTRO to an installed distro."
    fi

    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/$ROS_DISTRO_DETECTED/setup.bash"
    set -u
    log "Using ROS 2 distro: $ROS_DISTRO_DETECTED"
}

setup_osrf_repo() {
    install_apt_packages ca-certificates gnupg lsb-release wget

    if [ ! -s /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
        log "Adding OSRF package signing key..."
        wget -qO- https://packages.osrfoundation.org/gazebo.gpg \
            | "${SUDO[@]}" gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    fi

    log "Configuring OSRF Gazebo apt repository for $OS_CODENAME..."
    echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $OS_CODENAME main" \
        | "${SUDO[@]}" tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    APT_UPDATED=0
    apt_update_once
}

install_gazebo_harmonic() {
    if apt_package_installed gz-harmonic; then
        log "Gazebo Harmonic is already installed."
        return
    fi

    setup_osrf_repo

    if ! apt_package_available gz-harmonic; then
        die "gz-harmonic is not available for '$OS_CODENAME' from the configured OSRF repository."
    fi

    install_apt_packages gz-harmonic
}

select_first_available_package_set() {
    local pkg

    while [ "$#" -gt 0 ]; do
        local packages=()

        while [ "$#" -gt 0 ] && [ "$1" != "--" ]; do
            packages+=("$1")
            shift
        done

        if [ "$#" -gt 0 ] && [ "$1" = "--" ]; then
            shift
        fi

        local all_available=1
        for pkg in "${packages[@]}"; do
            if ! apt_package_available "$pkg"; then
                all_available=0
                break
            fi
        done

        if [ "$all_available" -eq 1 ]; then
            printf "%s\n" "${packages[@]}"
            return
        fi
    done

    return 1
}

install_ros_gz_bridge() {
    local versioned_bridge="ros-$ROS_DISTRO_DETECTED-ros-gzharmonic-bridge"
    local versioned_sim="ros-$ROS_DISTRO_DETECTED-ros-gzharmonic-sim"
    local generic_bridge="ros-$ROS_DISTRO_DETECTED-ros-gz-bridge"
    local generic_sim="ros-$ROS_DISTRO_DETECTED-ros-gz-sim"
    local selected_package_array=()
    local selected_packages

    if apt_package_installed "$versioned_bridge" || apt_package_installed "$generic_bridge"; then
        log "ROS/Gazebo bridge package is already installed."
        return
    fi

    apt_update_once

    if ! selected_packages="$(select_first_available_package_set \
        "$versioned_bridge" "$versioned_sim" -- \
        "$generic_bridge" "$generic_sim")"; then
        die "No ROS/Gazebo Harmonic bridge package set found for ROS '$ROS_DISTRO_DETECTED'. Tried '$versioned_bridge $versioned_sim' and '$generic_bridge $generic_sim'."
    fi

    mapfile -t selected_package_array <<< "$selected_packages"
    install_apt_packages "${selected_package_array[@]}"
}

clone_or_update_aeroloop() {
    local aeroloop_dir="$WORKSPACE_DIR/ext/aeroloop_gazebo"

    mkdir -p "$WORKSPACE_DIR/ext"

    if [ -d "$aeroloop_dir/.git" ]; then
        log "Updating aeroloop_gazebo..."
        git -C "$aeroloop_dir" fetch --all --prune
        git -C "$aeroloop_dir" checkout gz
        git -C "$aeroloop_dir" pull --ff-only
    elif [ -e "$aeroloop_dir" ] && [ "$(find "$aeroloop_dir" -mindepth 1 -maxdepth 1 | wc -l)" -gt 0 ]; then
        die "$aeroloop_dir exists but is not a git checkout. Move it aside or clone betaflight/aeroloop_gazebo there."
    else
        rm -rf "$aeroloop_dir"
        log "Cloning aeroloop_gazebo..."
        git clone -b gz https://github.com/betaflight/aeroloop_gazebo.git "$aeroloop_dir"
    fi

    [ -x "$aeroloop_dir/build_plugin.sh" ] || die "Missing executable build_plugin.sh in $aeroloop_dir."

    log "Building aeroloop_gazebo plugin..."
    (cd "$aeroloop_dir" && ./build_plugin.sh)
}

clone_or_update_betaflight() {
    local bf_dir="$WORKSPACE_DIR/ext/anemos-betaflight"

    mkdir -p "$WORKSPACE_DIR/ext"

    if [ -d "$bf_dir/.git" ]; then
        log "Updating anemos-betaflight..."
        git -C "$bf_dir" fetch --all --prune
        git -C "$bf_dir" pull --ff-only
    elif [ -e "$bf_dir" ] && [ "$(find "$bf_dir" -mindepth 1 -maxdepth 1 | wc -l)" -gt 0 ]; then
        die "$bf_dir exists but is not a git checkout. Move it aside or clone ETHRoboticsClub/anemos-betaflight there."
    else
        rm -rf "$bf_dir"
        log "Cloning anemos-betaflight..."
        git clone https://github.com/ETHRoboticsClub/anemos-betaflight.git "$bf_dir"
    fi
}

build_betaflight_sitl() {
    local bf_dir="$WORKSPACE_DIR/ext/anemos-betaflight"
    local sitl_bin

    [ -d "$bf_dir" ] || die "ext/anemos-betaflight not found. Clone the custom Betaflight source into $bf_dir."
    [ -f "$bf_dir/Makefile" ] || die "$bf_dir does not look like a Betaflight checkout because Makefile is missing."

    log "Building Betaflight SITL binary..."
    log "Initializing Betaflight submodules sequentially..."
    (cd "$bf_dir" && git submodule update --init --recursive) >&2

    (cd "$bf_dir"; make clean TARGET=SITL || true) >&2
    (cd "$bf_dir" && make TARGET=SITL "-j$(nproc)") >&2

    sitl_bin="$(find "$bf_dir/obj" -name "betaflight_*_SITL*" -type f -executable 2>/dev/null | head -n 1 || true)"
    if [ -z "$sitl_bin" ]; then
        sitl_bin="$bf_dir/obj/main/betaflight_SITL"
    fi

    [ -x "$sitl_bin" ] || die "Betaflight SITL binary not found or not executable after build."

    echo "$sitl_bin"
}

load_betaflight_config() {
    local sitl_bin="$1"
    local config_file="$WORKSPACE_DIR/docs/docs/files/betaflight_conf/sitl_config.txt"

    log "Loading Betaflight SITL configuration..."
    if [ -f "$config_file" ]; then
        (cd "$WORKSPACE_DIR" && "$sitl_bin" --config "$config_file")
        log "Betaflight SITL configuration loaded and saved to eeprom.bin."
    else
        log "Warning: Betaflight config file not found at $config_file"
    fi
}

build_ros_workspace() {
    require_cmd colcon

    log "Building ROS 2 workspace packages..."
    mkdir -p "$WORKSPACE_DIR/ext"
    echo "# Tell colcon to ignore this directory" > "$WORKSPACE_DIR/ext/COLCON_IGNORE"

    validate_colcon_package_sources "${COLCON_PACKAGES[@]}"
    cleanup_stale_colcon_symlinks

    (cd "$WORKSPACE_DIR" && colcon build --symlink-install --packages-up-to "${COLCON_PACKAGES[@]}")
}

main() {
    local sitl_bin

    log "Starting Betaflight SITL + Gazebo setup."
    log "Workspace directory: $WORKSPACE_DIR"

    sudo_cmd
    detect_os
    source_ros_setup

    require_cmd apt-cache
    require_cmd apt-get
    require_cmd dpkg-query
    require_cmd git
    require_cmd make
    require_cmd nproc

    install_apt_packages build-essential cmake
    install_gazebo_harmonic
    install_ros_gz_bridge

    clone_or_update_aeroloop
    clone_or_update_betaflight
    sitl_bin="$(build_betaflight_sitl)"
    load_betaflight_config "$sitl_bin"
    build_ros_workspace

    log "Setup completed successfully."
    log "Betaflight SITL binary path: $sitl_bin"
    log "To run SITL: ros2 launch drone_control racing_bringup.launch.py sitl:=true"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
