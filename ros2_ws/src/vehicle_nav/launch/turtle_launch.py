#!/usr/bin/env python3
"""
Launch file for vehicle navigation with TurtleSim visualization

Usage:
  ros2 launch vehicle_nav turtle_launch.py
  ros2 launch vehicle_nav turtle_launch.py planner:=astar controller:=pid
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('vehicle_nav')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Launch arguments
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='astar',
        description='Path planner to use: astar or rrtstar'
    )
    
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Controller to use: pid, lqr, or mpc'
    )
    
    use_cbf_arg = DeclareLaunchArgument(
        'use_cbf',
        default_value='false',
        description='Enable CBF safety filter'
    )
    
    # Get launch configurations
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    use_cbf = LaunchConfiguration('use_cbf')
    
    # TurtleSim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )
    
    # TurtleSim bridge (replaces simulator and handles visualization)
    bridge_node = Node(
        package='vehicle_nav',
        executable='turtlesim_bridge',
        name='turtlesim_bridge',
        parameters=[config_file],
        output='screen'
    )
    
    # A* planner node
    astar_node = Node(
        package='vehicle_nav',
        executable='astar_node',
        name='astar_planner',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", planner, "' == 'astar'"]))
    )
    
    # RRT* planner node
    rrtstar_node = Node(
        package='vehicle_nav',
        executable='rrtstar_node',
        name='rrtstar_planner',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", planner, "' == 'rrtstar'"]))
    )
    
    # PID controller node
    pid_node = Node(
        package='vehicle_nav',
        executable='pid_node',
        name='pid_controller',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"]))
    )
    
    # LQR controller node
    lqr_node = Node(
        package='vehicle_nav',
        executable='lqr_node',
        name='lqr_controller',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'lqr'"]))
    )
    
    # MPC controller node
    mpc_node = Node(
        package='vehicle_nav',
        executable='mpc_node',
        name='mpc_controller',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"]))
    )
    
    # CBF safety filter node
    cbf_node = Node(
        package='vehicle_nav',
        executable='cbf_node',
        name='cbf_filter',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(use_cbf)
    )
    
    # Goal publisher utility (not launched, run manually)
    # ros2 run vehicle_nav goal_publisher <x> <y> [theta]
    
    return LaunchDescription([
        planner_arg,
        controller_arg,
        use_cbf_arg,
        turtlesim_node,
        bridge_node,
        astar_node,
        rrtstar_node,
        pid_node,
        lqr_node,
        mpc_node,
        cbf_node,
    ])
