#!/bin/bash

usage() {
  echo "Usage: $0 [--rt_factor <value>] [--ros-version <ros2|ros1>] [--urdf_path <path>] [--headless] [--pub-rostime]"
  exit 1
}

RT_FACTOR=1.0
XMJ_ROS_VERSION="${XMJ_ROS_VERSION:-ros2}"
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-jazzy}"
RUNTIME_DIR="${XMJ_RUNTIME_DIR:-/tmp/TalosHybridMPC}"
URDF_PATH="${XMJ_URDF_PATH:-${RUNTIME_DIR}/talos.urdf}"
SRDF_PATH="${XMJ_SRDF_PATH:-${RUNTIME_DIR}/talos.srdf}"
XBOT_CONFIG_PATH="${XMJ_XBOT_CONFIG_PATH:-${RUNTIME_DIR}/xbot2_basic.yaml}"
HEADLESS=false
PUB_ROSTIME=false
TALOS_DESCRIPTION_ROOT="${TALOS_DESCRIPTION_ROOT:-${HOME}/ibrido_ws/src/talos-description/talos_description}"
TALOS_DESCRIPTION_REPO="$(dirname "$TALOS_DESCRIPTION_ROOT")"
WS_SRC_ROOT="$(dirname "$TALOS_DESCRIPTION_REPO")"
TALOS_URDF_XACRO="${TALOS_URDF_XACRO:-${TALOS_DESCRIPTION_ROOT}/robots/talos_full_v2.urdf.xacro}"
TALOS_SOURCE_SRDF_PATH="${TALOS_SOURCE_SRDF_PATH:-${TALOS_DESCRIPTION_ROOT}/srdf/talos.srdf}"
TALOS_XMJ_DIR="${TALOS_XMJ_DIR:-${HOME}/ibrido_ws/src/TalosHybridMPC/taloshybridmpc/config/xmj_env_files}"

require_valid_xml() {
  local path="$1"
  local root_tag="$2"

  if [ ! -s "$path" ]; then
    echo "XML file not found or empty: $path"
    exit 2
  fi

  if ! grep -Eq "<${root_tag}([[:space:]>])" "$path"; then
    echo "XML file does not contain a <${root_tag}> root: $path"
    exit 2
  fi
}

generate_talos_urdf() {
  local urdf_path="$1"

  if ! command -v xacro >/dev/null 2>&1; then
    echo "Cannot generate URDF: xacro is not available on PATH."
    exit 2
  fi

  if [ ! -f "$TALOS_URDF_XACRO" ]; then
    echo "Cannot generate URDF: xacro file not found: $TALOS_URDF_XACRO"
    exit 2
  fi

  mkdir -p "$(dirname "$urdf_path")"
  echo "Generating Talos URDF at $urdf_path"
  xacro "$TALOS_URDF_XACRO" \
    root:="$TALOS_DESCRIPTION_ROOT" \
    talos_description_inertial_root:="${TALOS_DESCRIPTION_REPO}/talos_description_inertial" \
    talos_description_calibration_root:="${TALOS_DESCRIPTION_REPO}/talos_description_calibration" \
    pal_urdf_utils_root:="${WS_SRC_ROOT}/pal_urdf_utils" \
    foot_collision:=thinbox \
    head_type:=default \
    flexibility:=False \
    test:=false \
    use_fixed_base:=false \
    use_sim:=true \
    enable_crane:=false \
    disable_gazebo_camera:=true \
    use_capsule_collision:=false \
    multiple:=false \
    gazebo_version:=classic \
    include_gazebo:=false \
    include_ros2_control:=false \
    include_head_sensors:=false \
    include_torso_imu:=false \
    include_grippers:=false \
    floating_joint:=true \
    use_abs_mesh_paths:=true \
    use_local_filesys_for_meshes:=false \
    -o "$urdf_path"
}

patch_runtime_xbot_config() {
  python3 -c 'from pathlib import Path
import sys
cfg = Path(sys.argv[1])
text = cfg.read_text()
text = text.replace("urdf_path: $PWD/talos.urdf", "urdf_path: " + sys.argv[2])
text = text.replace("srdf_path: $PWD/talos.srdf", "srdf_path: " + sys.argv[3])
text = text.replace("sim: $PWD/hal/talos_gz.yaml", "sim: " + sys.argv[4])
text = text.replace("dummy: $PWD/hal/talos_dummy.yaml", "dummy: " + sys.argv[5])
cfg.write_text(text)
' "$XBOT_CONFIG_PATH" "$URDF_PATH" "$SRDF_PATH" "${RUNTIME_DIR}/hal/talos_gz.yaml" "${RUNTIME_DIR}/hal/talos_dummy.yaml"
}

prepare_runtime_files() {
  mkdir -p "$RUNTIME_DIR"
  generate_talos_urdf "$URDF_PATH"
  require_valid_xml "$URDF_PATH" robot

  cp "$TALOS_SOURCE_SRDF_PATH" "$SRDF_PATH"
  require_valid_xml "$SRDF_PATH" robot

  cp "$TALOS_XMJ_DIR/xbot2_basic.yaml" "$XBOT_CONFIG_PATH"
  rm -rf "$RUNTIME_DIR/hal"
  cp -r "$TALOS_XMJ_DIR/hal" "$RUNTIME_DIR/hal"
  patch_runtime_xbot_config
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rt_factor) RT_FACTOR="$2"; shift ;;
    --ros-version|--ros_version) XMJ_ROS_VERSION="$2"; shift ;;
    --urdf_path|--urdf-path) URDF_PATH="$2"; shift ;;
    --headless) HEADLESS=true ;;
    --pub-rostime|--pub_rostime) PUB_ROSTIME=true ;;
    *) echo "Unknown arg: $1"; usage ;;
  esac
  shift
done

source "${HOME}/ibrido_utils/mamba_utils/bin/_activate_current_env.sh"
micromamba activate ibrido
if [ -f /opt/xbot/setup.sh ]; then
  source /opt/xbot/setup.sh
fi

extra_args=()
if [ "$HEADLESS" = true ]; then
  extra_args+=(--headless)
fi
if [ "$PUB_ROSTIME" = true ]; then
  case "$XMJ_ROS_VERSION" in
    1) XMJ_ROS_VERSION="ros1"; source "/opt/ros/${ROS1_DISTRO}/setup.bash" ;;
    2) XMJ_ROS_VERSION="ros2"; source "/opt/ros/${ROS2_DISTRO}/setup.bash" ;;
    ros1) source "/opt/ros/${ROS1_DISTRO}/setup.bash" ;;
    ros2) source "/opt/ros/${ROS2_DISTRO}/setup.bash" ;;
    *) echo "Unsupported ROS version: ${XMJ_ROS_VERSION}"; usage ;;
  esac
  extra_args+=(--pub_rostime --ros-version "$XMJ_ROS_VERSION")
fi
source "${HOME}/ibrido_ws/setup.bash"

prepare_runtime_files

python "${HOME}/ibrido_ws/src/xbot2_mujoco/tests/PyXBotMjSim/launch_simulator.py" --urdf_path "$URDF_PATH" \
    --simopt_path "${TALOS_XMJ_DIR}/sim_opt.xml" \
    --world_path "${TALOS_XMJ_DIR}/world.xml" \
    --sites_path "${TALOS_XMJ_DIR}/sites.xml" \
    --xbot_config_path "$XBOT_CONFIG_PATH" \
    --blink_name base_link \
    --rt_factor "$RT_FACTOR" \
    "${extra_args[@]}"
