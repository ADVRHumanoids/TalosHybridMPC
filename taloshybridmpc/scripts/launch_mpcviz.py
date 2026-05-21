#!/usr/bin/env python
import argparse

from mpc_viz.MPCViz import MPCViz
from mpc_viz.utils.sys_utils import PathsGetter

from taloshybridmpc.utils.talos_urdf_gen import TalosUrdfGen


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Launch MPCViz for Talos.")
    parser.add_argument(
        "--ns",
        type=str,
        default="talos",
        help="Shared-memory namespace used by the Talos runtime.")
    parser.add_argument(
        "--dpath",
        type=str,
        default="/root/ibrido_ws/src/talos-description/talos_description",
        help="Path to the talos_description package.")
    parser.add_argument(
        "--xacro_name",
        type=str,
        default="talos_full_v2",
        help="Name of the xacro under talos_description/robots, without .urdf.xacro.")
    parser.add_argument(
        "--robotname",
        type=str,
        default="talos",
        help="Name used for the generated MPCViz URDF.")
    parser.add_argument(
        "--nodes_perc",
        type=int,
        default=30,
        help="Percentage of MPC horizon nodes to display.")
    parser.add_argument(
        "--base_link_name",
        type=str,
        default="base_link",
        help="Root link name used by MPCViz transforms.")
    parser.add_argument(
        "--show_heightmap",
        action="store_true",
        help="Visualize heightmap markers if available.")
    parser.add_argument(
        "--no_check_jnt_names",
        action="store_true",
        help="Skip MPCViz joint-name consistency checks.")

    args = parser.parse_args()

    syspaths = PathsGetter()

    urdf_generator = TalosUrdfGen(
        descr_path=args.dpath,
        robotname=args.robotname,
        xacro_name=args.xacro_name,
        floating_joint=False,
        name="talosUrdfMPCViz")

    mpc_viz = MPCViz(
        urdf_file_path=urdf_generator.urdf_path,
        rviz_config_path=syspaths.DEFAULT_RVIZ_CONFIG_PATH,
        namespace=args.ns,
        basename="MPCViz",
        rate=100,
        use_only_collisions=False,
        check_jnt_names=not args.no_check_jnt_names,
        nodes_perc=args.nodes_perc,
        base_link_name=args.base_link_name,
        show_heightmap=args.show_heightmap)

    mpc_viz.run()
