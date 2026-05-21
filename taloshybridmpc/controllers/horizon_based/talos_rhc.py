from typing import Dict

import numpy as np

from aug_mpc.controllers.rhc.horizon_based.hybrid_quad_rhc import HybridQuadRhc
from aug_mpc.controllers.rhc.horizon_based.horizon_imports import ContactTask
from mpc_hive.utilities.math_utils import world2base_frame

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

        custom_opts = dict(custom_opts or {})
        custom_opts.setdefault("initial_force_load_divisor", 2.0)

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
        self._contact_vertex_frames = {}
        self._fk_cache = {}
        self._contact_torque_base_loc_aux = np.zeros((1, 3), dtype=self._dtype)

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

        self._contact_vertex_frames = self._build_contact_vertex_frames()

    def _build_contact_vertex_frames(self):
        contact_vertex_frames = {}
        for task in self._ti.task_list:
            if not isinstance(task, ContactTask):
                continue

            vertex_frames = []
            for interaction_task in task.dynamics_tasks:
                if hasattr(interaction_task, "vertex_frames"):
                    vertex_frames.extend(interaction_task.vertex_frames)

            if vertex_frames:
                contact_vertex_frames[task.getName()] = vertex_frames

        for contact_name, forces in self._model.cmap.items():
            if contact_name in contact_vertex_frames:
                continue
            contact_vertex_frames[contact_name] = [
                force.getName().removeprefix("f_")
                for force in forces
            ]

        return contact_vertex_frames

    def _get_contact_names(self):
        if hasattr(self, "_model") and self._model is not None:
            return list(self._model.cmap.keys())
        return super()._get_contact_names()

    def _sanitize_force(self, data):
        np.nan_to_num(data, nan=1e6, posinf=1e6, neginf=-1e6, copy=False)
        np.clip(a=data, a_max=1e6, a_min=-1e6, out=data)
        return data

    def _get_f_from_sol(self):
        try:
            data = []
            for contact_name in self._get_contact_names():
                contact_force = None
                for force in self._model.cmap[contact_name]:
                    force_data = self._ti.solution[force.getName()].astype(self._dtype)
                    self._sanitize_force(force_data)
                    if contact_force is None:
                        contact_force = np.zeros_like(force_data)
                    contact_force += force_data
                data.append(contact_force)
            return np.concatenate(data, axis=0)
        except Exception:
            return None

    def _fk_pos(self, frame_name: str, q):
        if frame_name not in self._fk_cache:
            self._fk_cache[frame_name] = self._kin_dyn.fk(frame_name)
        return np.array(
            self._fk_cache[frame_name](q=q)["ee_pos"],
            dtype=self._dtype).reshape(3, 1)

    def _get_t_from_sol(self, node_idx: int):
        try:
            q = self._ti.solution["q"][:, node_idx:node_idx + 1]
            data = []

            for contact_name in self._get_contact_names():
                contact_pos = self._fk_pos(contact_name, q)
                contact_torque = np.zeros((3, 1), dtype=self._dtype)
                vertex_frames = self._contact_vertex_frames[contact_name]

                for force, vertex_frame in zip(self._model.cmap[contact_name], vertex_frames):
                    force_data = self._ti.solution[force.getName()][:, node_idx:node_idx + 1].astype(self._dtype)
                    self._sanitize_force(force_data)
                    vertex_pos = self._fk_pos(vertex_frame, q)
                    contact_torque += np.cross(
                        (vertex_pos - contact_pos).reshape(3),
                        force_data.reshape(3)).reshape(3, 1)

                self._sanitize_force(contact_torque)
                data.append(contact_torque)

            return np.concatenate(data, axis=0)
        except Exception:
            return None

    def _write_cmds_from_sol(self):
        super()._write_cmds_from_sol()

        node_idx_f_estimate = self._rhc_cmds_node_idx - 1
        t_contact = self._get_t_from_sol(node_idx=node_idx_f_estimate)
        if t_contact is None:
            return

        contact_names = self.robot_state.contact_names()
        rhc_q_estimate = self._get_root_full_q_from_sol(
            node_idx=node_idx_f_estimate)[:, 3:7]

        for i, contact in enumerate(contact_names[:len(self._get_contact_names())]):
            contact_idx = i * 3
            contact_torque_rhc_world = t_contact[
                contact_idx:contact_idx + 3,
                :].T
            world2base_frame(
                v_w=contact_torque_rhc_world,
                q_b=rhc_q_estimate,
                v_out=self._contact_torque_base_loc_aux,
                is_q_wijk=False)
            self.robot_cmds.contact_wrenches.set(
                data=self._contact_torque_base_loc_aux,
                data_type="t",
                robot_idxs=self.controller_index_np,
                contact_name=contact)

        self.robot_cmds.contact_wrenches.synch_retry(
            row_index=self.controller_index,
            col_index=0,
            row_index_view=0,
            n_rows=1,
            n_cols=self.robot_cmds.contact_wrenches.n_cols,
            read=False)
