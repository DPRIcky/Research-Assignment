#!/usr/bin/env python3
"""
Launch file for vehicle navigation system
Launches simulator, RViz2, and selected planner/controller/safety nodes

Usage:
  ros2 launch vehicle_nav sim_launch.py
  ros2 launch vehicle_nav sim_launch.py planner:=astar controller:=pid
  ros2 launch vehicle_nav sim_launch.py planner:=rrtstar controller:=lqr use_cbf:=false
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
    rviz_config = os.path.join(pkg_dir, 'config', 'vehicle_nav.rviz')
    
    # Get TurtleBot3 description (using burger model)
    try:
        tb3_dir = get_package_share_directory('turtlebot3_description')
        urdf_file = os.path.join(tb3_dir, 'urdf', 'turtlebot3_burger.urdf')
    except:
        # Fallback if turtlebot3_description not installed
        urdf_file = None
    
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
        default_value='true',
        description='Enable CBF safety filter'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    use_robot_model_arg = DeclareLaunchArgument(
        'use_robot_model',
        default_value='false',
        description='Use TurtleBot3 robot model (requires turtlebot3_description)'
    )
    
    # Get launch configurations
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    use_cbf = LaunchConfiguration('use_cbf')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_model = LaunchConfiguration('use_robot_model')
    
    # Simulator node
    simulator_node = Node(
        package='vehicle_nav',
        executable='simulator_node',
        name='vehicle_simulator',
        parameters=[config_file],
        output='screen'
    )
    
    # Robot state publisher (for TurtleBot3 visualization - optional)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[urdf_file] if urdf_file else [],
        condition=IfCondition(use_robot_model)
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # Planner nodes (conditionally launch based on planner argument)
    astar_node = Node(
        package='vehicle_nav',
        executable='astar_node',
        name='astar_planner',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", planner, "' == 'astar'"]))
    )
    
    rrtstar_node = Node(
        package='vehicle_nav',
        executable='rrtstar_node',
        name='rrtstar_planner',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", planner, "' == 'rrtstar'"]))
    )
    
    # Controller nodes (conditionally launch based on controller argument)
    pid_node = Node(
        package='vehicle_nav',
        executable='pid_node',
        name='pid_controller',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"]))
    )
    
    lqr_node = Node(
        package='vehicle_nav',
        executable='lqr_node',
        name='lqr_controller',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'lqr'"]))
    )
    
    mpc_node = Node(
        package='vehicle_nav',
        executable='mpc_node',
        name='mpc_controller',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"]))
    )
    
    # CBF safety filter (conditionally launch)
    cbf_node = Node(
        package='vehicle_nav',
        executable='cbf_node',
        name='cbf_safety_filter',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(use_cbf)
    )
    
    return LaunchDescription([
        planner_arg,
        controller_arg,
        use_cbf_arg,
        use_robot_model_arg,
        use_rviz_arg,
        simulator_node,
        robot_state_publisher,
        rviz_node,
        astar_node,
        rrtstar_node,
        pid_node,
        lqr_node,
        mpc_node,
        cbf_node,
    ])
