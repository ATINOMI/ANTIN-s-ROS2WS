"""只读加载已有地图，定位与导航，不运行全局建图。"""
from ament_index_python.packages import get_package_share_directory
from mini_nav_fastlivo.bringup import description


def generate_launch_description():
    return description(False, get_package_share_directory('mini_nav_bringup'))
