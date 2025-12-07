#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    总 bringup：

      1. 立即启动硬件 bringup：
           iqr_tb4_bringup/launch/bringup.launch.py
         （假设里面已经负责启动底盘、雷达、nav2、localization 等）

      2. 延时若干秒后，启动：
           - tb4_autonav/waypoint_navigator
           - tb4_autonav/traffic_detector

      WaypointNavigator 内部会做系统自检：
        - nav2 action
        - /odom
        - /traffic_event (vision ready)
      全部 OK 后才开始任务执行。
    """

    # 1) 硬件 bringup
    iqr_tb4_share = get_package_share_directory("iqr_tb4_bringup")
    hw_bringup_path = os.path.join(
        iqr_tb4_share,
        "launch",
        "bringup.launch.py",
    )

    hw_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_bringup_path)
    )

    # 2) WaypointNavigator 节点
    waypoint_node = Node(
        package="tb4_autonav",
        executable="waypoint_navigator",
        name="waypoint_navigator",
        output="screen",
    )

    # 3) TrafficDetector 节点
    traffic_node = Node(
        package="tb4_autonav",
        executable="traffic_detector",
        name="traffic_detector",
        output="screen",
    )

    ld = LaunchDescription()

    # 先启动硬件
    ld.add_action(hw_bringup)

    # 若硬件 bringup 比较慢，可以适当增大延时（比如 10s / 15s）
    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[waypoint_node],
        )
    )

    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[traffic_node],
        )
    )

    return ld
