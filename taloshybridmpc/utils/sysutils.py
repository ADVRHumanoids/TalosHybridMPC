import os


class PathsGetter:

    def __init__(self):

        self.ROOT_DIR = os.path.dirname(os.path.dirname(__file__))

        self.CONTROLLER_ROOT_DIR = os.path.join(
            self.ROOT_DIR,
            "controllers",
            "horizon_based")

        self.CONTROLLER_CFGS_DIR = os.path.join(
            self.CONTROLLER_ROOT_DIR,
            "cfgs")

        self.RHCCONFIGPATH = os.path.join(
            self.CONTROLLER_CFGS_DIR,
            "talos_rhc_config")

        self.JNT_IMP_CONFIG = os.path.join(
            self.ROOT_DIR,
            "config",
            "jnt_imp_config")

        self.JNT_IMP_CONFIG_XBOT = os.path.join(
            self.ROOT_DIR,
            "config",
            "xmj_env_files",
            "xbot2_basic")


if __name__ == "__main__":
    paths = PathsGetter()
    print(paths.ROOT_DIR)
    print(paths.CONTROLLER_ROOT_DIR)
    print(paths.RHCCONFIGPATH)
