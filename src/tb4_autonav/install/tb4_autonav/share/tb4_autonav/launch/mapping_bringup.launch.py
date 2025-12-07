from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    建图流程整合版：
      1. 硬件启动：iqr_tb4_bringup/bringup.launch.py
      2. SLAM：turtlebot4_navigation/slam.launch.py
      3. 可视化：turtlebot4_viz/view_robot.launch.py
    """

    # 1) 硬件 bringup
    iqr_tb4_bringup_share = get_package_share_directory('iqr_tb4_bringup')
    bringup_launch_path = os.path.join(
        iqr_tb4_bringup_share,
        'launch',
        'bringup.launch.py'
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path)
    )

    # 2) SLAM
    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    tb4_nav_share = get_package_share_directory('tb4_autonav')
    slam_launch_path = os.path.join(
        tb4_nav_share,
        'launch',
        'slam.launch.py'
    )
    slam_para = os.path.join(
        tb4_nav_share,
        'config',
        'slam_with_init.yaml'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'params': slam_para,   # 关键：把参数文件传给 slam.launch.py
            # 如果你有命名空间也可以一起传，例如：
            # 'namespace': '/tb04',
        }.items()
    )

    # 3) RViz 可视化（view_robot）
    tb4_viz_share = get_package_share_directory('turtlebot4_viz')
    view_robot_launch_path = os.path.join(
        tb4_viz_share,
        'launch',
        'view_robot.launch.py'
    )

    view_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(view_robot_launch_path)
    )

    ld = LaunchDescription()
    ld.add_action(bringup)
    ld.add_action(slam)
    ld.add_action(view_robot)

    return ld
