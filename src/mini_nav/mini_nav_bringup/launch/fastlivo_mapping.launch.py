"""启动手动建图，不启动自主导航。"""
from ament_index_python.packages import get_package_share_directory
from mini_nav_fastlivo.bringup import description


def generate_launch_description():
    return description(True, get_package_share_directory('mini_nav_bringup'))
