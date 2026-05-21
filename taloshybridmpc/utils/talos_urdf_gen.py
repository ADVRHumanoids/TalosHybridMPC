import os
from typing import List

from aug_mpc.utils.xrdf_gen import generate_urdf

from taloshybridmpc.utils.xrdf_cmd import get_xrdf_cmds_talos


class TalosUrdfGen:

    def __init__(self,
            descr_path: str,
            robotname: str = "talos",
            xacro_name: str = "talos_full_v2",
            name: str = "TalosUrdfMPCViz",
            floating_joint: bool = False,
            custom_args_xacro: List[str] = None):

        self.robotname = robotname
        self.xacro_name = xacro_name
        self.output_path = "/tmp"
        self.name = name
        self.descr_dump_path = os.path.join(self.output_path, name)
        self.descr_path = descr_path
        self.floating_joint = floating_joint
        self.custom_args_xacro = custom_args_xacro
        self.generated = False
        self.urdf_path = ""

        self.generate_urdf()

    def generate_urdf(self):
        os.makedirs(self.descr_dump_path, exist_ok=True)
        xacro_path = os.path.join(
            self.descr_path,
            "robots",
            f"{self.xacro_name}.urdf.xacro")

        self.urdf_path = generate_urdf(
            robot_name=self.robotname,
            xacro_path=xacro_path,
            dump_path=self.descr_dump_path,
            xrdf_cmds=self._xrdf_cmds())

        print(f"[TalosUrdfGen]: generated URDF at {self.urdf_path}")
        self.generated = True

    def _xrdf_cmds(self):
        return get_xrdf_cmds_talos(
            urdf_descr_root_path=self.descr_path,
            floating_joint=self.floating_joint,
            use_local_filesys_for_meshes=True,
            custom_args_xacro=self.custom_args_xacro)
