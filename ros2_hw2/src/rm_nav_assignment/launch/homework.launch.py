# -*- coding: utf-8 -*-
"""一键启动：地图 + 规划节点 + RVIZ

 ros2 launch rm_nav_assignment homework.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rm_nav_assignment')
    map_pgm = os.path.join(pkg, 'map', 'mymap.pgm')
    params = os.path.join(pkg, 'config', 'planner_params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'homework.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RVIZ'),

        # 规划节点：自己读 mymap.pgm，发布 /map、/global_costmap/costmap、/planned_path
        Node(
            package='rm_nav_assignment',
            executable='planner_node',
            name='rm_nav_planner',
            output='screen',
            parameters=[params, {'map_file': map_pgm}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
