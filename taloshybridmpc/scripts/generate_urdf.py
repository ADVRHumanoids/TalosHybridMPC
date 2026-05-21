#!/usr/bin/env python
import argparse

from taloshybridmpc.utils.talos_urdf_gen import TalosUrdfGen


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Generate the Talos URDF used by MPCViz.")
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
        help="Name used for the generated URDF file.")
    parser.add_argument(
        "--floating_joint",
        action="store_true",
        help="Generate the URDF with a root floating joint.")

    args = parser.parse_args()

    urdf_generator = TalosUrdfGen(
        descr_path=args.dpath,
        robotname=args.robotname,
        xacro_name=args.xacro_name,
        floating_joint=args.floating_joint,
        name=args.robotname + "Urdf")

    print(urdf_generator.urdf_path)
