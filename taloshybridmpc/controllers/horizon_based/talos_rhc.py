from typing import Dict

import numpy as np

from aug_mpc.controllers.rhc.horizon_based.hybrid_quad_rhc import HybridQuadRhc

from taloshybridmpc.utils.sysutils import PathsGetter


class TalosRhc(HybridQuadRhc):

    def __init__(self,
            srdf_path: str,
            urdf_path: str,
            robot_name: str,
            codegen_dir: str,
            n_nodes: float = 31,
            dt: float = 0.03,
            injection_node: int = 10,
            max_solver_iter=1,
            open_loop: bool = True,
            close_loop_all: bool = False,
            dtype=np.float32,
            verbose=False,
            debug=False,
            refs_in_hor_frame=True,
            timeout_ms: int = 60000,
            custom_opts: Dict = {}):

        paths = PathsGetter()
        self._files_suffix = "_open" if open_loop else ""
        config_path = paths.RHCCONFIGPATH + self._files_suffix + ".yaml"

        super().__init__(
            srdf_path=srdf_path,
            urdf_path=urdf_path,
            config_path=config_path,
            robot_name=robot_name,
            codegen_dir=codegen_dir,
            n_nodes=n_nodes,
            dt=dt,
            injection_node=injection_node,
            max_solver_iter=max_solver_iter,
            open_loop=open_loop,
            close_loop_all=close_loop_all,
            dtype=dtype,
            verbose=verbose,
            debug=debug,
            refs_in_hor_frame=refs_in_hor_frame,
            timeout_ms=timeout_ms,
            custom_opts=custom_opts)

        self._fail_idx_scale = 1e-9
        self._fail_idx_thresh_open_loop = 1e0
        self._fail_idx_thresh_close_loop = 10
        self._fail_idx_thresh = (
            self._fail_idx_thresh_open_loop
            if open_loop
            else self._fail_idx_thresh_close_loop)

        self._rhc_fpaths.append(paths.JNT_IMP_CONFIG + ".yaml")

    def _set_rhc_pred_idx(self):
        self._pred_node_idx = round((self._n_nodes - 1) * 2 / 3)

    def _set_rhc_cmds_idx(self):
        self._rhc_cmds_node_idx = 2

    def _config_override(self):
        if "add_upper_body" not in self._custom_opts:
            self._custom_opts["add_upper_body"] = False

    def _init_problem(self):

        fixed_jnts_patterns = []
        if not self._custom_opts.get("add_upper_body", False):
            fixed_jnts_patterns += [
                "arm_left",
                "arm_right",
                "gripper_left",
                "gripper_right",
                "head",
                "torso",
            ]

        flight_duration_sec = 0.5
        flight_duration = int(flight_duration_sec / self._dt)
        post_flight_duration_sec = 0.2
        post_flight_duration = int(post_flight_duration_sec / self._dt)

        step_height = self._custom_opts.get("step_height", 0.08)

        super()._init_problem(
            fixed_jnt_patterns=fixed_jnts_patterns,
            wheels_patterns=[],
            foot_linkname="left_sole_link",
            flight_duration=flight_duration,
            post_flight_stance=post_flight_duration,
            step_height=step_height,
            keep_yaw_vert=True,
            yaw_vertical_weight=25.0,
            vertical_landing=True,
            vertical_land_weight=10.0,
            phase_force_reg=2e-2,
            vel_bounds_weight=1.0)
