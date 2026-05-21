from typing import List


def get_xrdf_cmds_horizon(
        urdf_descr_root_path: str = None,
        custom_args_xacro: List[str] = None):

    return get_xrdf_cmds_horizon_talos(
        urdf_descr_root_path=urdf_descr_root_path,
        custom_args_xacro=custom_args_xacro)


def get_xrdf_cmds_horizon_talos(
        urdf_descr_root_path: str = None,
        custom_args_xacro: List[str] = None):

    cmds = [
        "foot_collision:=thinbox",
        "head_type:=default",
        "flexibility:=False",
        "use_fixed_base:=false",
        "use_sim:=true",
        "enable_crane:=false",
        "disable_gazebo_camera:=true",
        "use_capsule_collision:=false",
        "multiple:=false",
        "gazebo_version:=classic",
    ]

    if custom_args_xacro:
        cmds += custom_args_xacro

    return cmds
