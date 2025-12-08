#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    导航 + 自动任务 bringup：

      1. 立即启动硬件：iqr_tb4_bringup/bringup.launch.py
      2. 启动 localization（带自己的 params + map）
      3. 稍后启动 Nav2（带 nav2.yaml、自身的 autostart 等）
      4. 再启动自动导航任务：
           - tb4_autonav/task_combined_navigator
           - tb4_autonav/traffic_detector_node
      5. （可选）启动 RViz view_robot
    """

    # ========= 基本路径 =========
    iqr_tb4_bringup_share = get_package_share_directory('iqr_tb4_bringup')
    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    tb4_viz_share = get_package_share_directory('turtlebot4_viz')

    bringup_launch_path = os.path.join(
        iqr_tb4_bringup_share, 'launch', 'bringup.launch.py'
    )
    loca_launch_path = os.path.join(
        tb4_nav_share, 'launch', 'localization.launch.py'
    )
    nav2_launch_path = os.path.join(
        tb4_nav_share, 'launch', 'nav2.launch.py'
    )
    view_robot_launch_path = os.path.join(
        tb4_viz_share, 'launch', 'view_robot.launch.py'
    )

    # ========= 声明我们自己用的 LaunchArguments =========
    # 注意：这里用 *不同的名字*，避免和子 launch 的 use_sim_time/params_file 等冲突

    map_yaml = LaunchConfiguration('map')
    loca_params = LaunchConfiguration('loca_params')
    nav2_params = LaunchConfiguration('nav2_params')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/tony/ee211_maps/test1125.yaml',
        description='地图 yaml 路径（给 localization 用）',
    )

    declare_loca_params = DeclareLaunchArgument(
        'loca_params',
        # 默认用原装的 localization.yaml
        default_value=os.path.join(tb4_nav_share, 'config', 'localization.yaml'),
        description='Localization 参数文件',
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        # 默认用原装的 nav2.yaml
        default_value=os.path.join(tb4_nav_share, 'config', 'nav2.yaml'),
        description='Nav2 参数文件',
    )

    # ========= 1) 硬件 bringup =========
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path)
    )

    # ========= 2) Localization（明确传 map + 自己的 params key）=========
    loca = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(loca_launch_path),
        launch_arguments={
            'map': map_yaml,
            # turtlebot4_navigation 的 localization.launch.py 一般用 "params"
            'params': loca_params,
            # 如有需要，这里也可传 use_sim_time
            # 'use_sim_time': 'false',
        }.items(),
    )

    # ========= 3) Nav2（明确传 nav2.yaml + use_sim_time/autostart）=========
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            # nav2.launch.py 用的是 "params_file"
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    # ========= 4) RViz view_robot（可选）=========
    view_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(view_robot_launch_path)
    )

    # ========= 5) 自动任务节点（tb4_autonav）=========

    task_combined_navigator = Node(
        package='tb4_autonav',
        executable='task_combined_navigator',
        name='task_combined_navigator',
        output='screen',
    )

    traffic_detector_node = Node(
        package='tb4_autonav',
        executable='traffic_detector_node',
        name='traffic_detector_node',
        output='screen',
    )

    # ========= 组装 LaunchDescription =========
    ld = LaunchDescription()

    # 先声明我们自己的参数
    ld.add_action(declare_map)
    ld.add_action(declare_loca_params)
    ld.add_action(declare_nav2_params)

    # 1. 最先起硬件
    ld.add_action(bringup)

    # 2. 稍后起 localization（给它一点时间加载 map 等）
    ld.add_action(
        TimerAction(
            period=4.0,
            actions=[loca],
        )
    )

    # 3. 再等一会儿起 Nav2（此时 localization 基本已经就绪）
    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[nav2],
        )
    )

    # 4. RViz 可以和 nav2 差不多同时起（你也可以注释掉）
    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[view_robot],
        )
    )

    ld.add_action(
        TimerAction(
            period=14.0,
            actions=[traffic_detector_node],
        )
    )

    return ld