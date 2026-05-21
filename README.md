# TalosHybridMPC

Talos-specific IBRIDO integration package.

This package mirrors the layout used by `CentauroHybridMPC` and contains the
Talos RHC cluster client, Horizon controller wrapper, xacro argument helpers,
and initial controller/joint-impedance configuration files.

The current implementation is an integration scaffold for a one-environment,
open-loop debug setup. The Talos Horizon problem still needs proper tuning and
validation before it should be treated as a production controller.

## MPCViz

Talos can be visualized through the standard IBRIDO ROS 2 bridge plus a
Talos-specific MPCViz launcher:

```bash
cd /root/ibrido_ws/src/AugMPC/aug_mpc/scripts
python utilities/launch_rhc2rviz_bridge.py --ros2 --rhc_refs_in_h_frame --ns talos --with_agent_refs --no_rhc_internal

cd /root/ibrido_ws/src/TalosHybridMPC/taloshybridmpc/scripts
python launch_mpcviz.py --ns talos --nodes_perc 10
```

The launcher generates a temporary URDF from `talos_full_v2.urdf.xacro` with
the IBRIDO Talos xacro options, no root floating joint, no grippers, and
`file://` mesh URIs for RViz.
