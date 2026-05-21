from typing import List
import os


def get_xrdf_cmds_horizon(
        urdf_descr_root_path: str = None,
        custom_args_xacro: List[str] = None):

    return get_xrdf_cmds_horizon_talos(
        urdf_descr_root_path=urdf_descr_root_path,
        custom_args_xacro=custom_args_xacro)


def get_xrdf_cmds_talos(
        urdf_descr_root_path: str = None,
        floating_joint: bool = False,
        use_local_filesys_for_meshes: bool = False,
        custom_args_xacro: List[str] = None):

    cmds = _get_xrdf_cmds_talos_common(
        urdf_descr_root_path=urdf_descr_root_path,
        floating_joint=floating_joint,
        use_local_filesys_for_meshes=use_local_filesys_for_meshes)

    if custom_args_xacro:
        cmds += custom_args_xacro

    return cmds


def get_xrdf_cmds_horizon_talos(
        urdf_descr_root_path: str = None,
        custom_args_xacro: List[str] = None):

    cmds = _get_xrdf_cmds_talos_common(
        urdf_descr_root_path=urdf_descr_root_path,
        floating_joint=True,
        use_local_filesys_for_meshes=False)

    if custom_args_xacro:
        cmds += custom_args_xacro

    return cmds


def _get_xrdf_cmds_talos_common(
        urdf_descr_root_path: str = None,
        floating_joint: bool = False,
        use_local_filesys_for_meshes: bool = False):

    cmds = [
        "foot_collision:=thinbox",
        "head_type:=default",
        "flexibility:=False",
        "test:=false",
        "use_fixed_base:=false",
        "use_sim:=false",
        "enable_crane:=false",
        "disable_gazebo_camera:=true",
        "use_capsule_collision:=false",
        "multiple:=false",
        "gazebo_version:=classic",
        "include_gazebo:=false",
        "include_ros2_control:=false",
        "include_head_sensors:=false",
        "include_torso_imu:=false",
        "include_grippers:=false",
        f"floating_joint:={str(floating_joint).lower()}",
        "use_abs_mesh_paths:=true",
        f"use_local_filesys_for_meshes:={str(use_local_filesys_for_meshes).lower()}",
    ]

    if urdf_descr_root_path is not None:
        talos_repo_root = os.path.dirname(urdf_descr_root_path)
        ws_src_root = os.path.dirname(talos_repo_root)
        cmds += [
            "root:=" + urdf_descr_root_path,
            "talos_description_inertial_root:=" + os.path.join(
                talos_repo_root, "talos_description_inertial"),
            "talos_description_calibration_root:=" + os.path.join(
                talos_repo_root, "talos_description_calibration"),
            "pal_urdf_utils_root:=" + os.path.join(
                ws_src_root, "pal_urdf_utils"),
        ]

    return cmds
