"""Start Gazebo with the sort cell, add the SCARA arm, connect it to ROS 2 and open RViz.

    ros2 launch scara_sort_sim sim.launch.py
    ros2 launch scara_sort_sim sim.launch.py gui:=false rviz:=false   # no windows at all

gui:=false runs Gazebo without its window (the camera still renders, off-screen).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = FindPackageShare('scara_sort_sim')
    world = PathJoinSubstitution([pkg, 'worlds', 'sort_cell.sdf'])
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'scara.urdf.xacro'])
    bridge_config = PathJoinSubstitution([pkg, 'config', 'bridge.yaml'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'sort.rviz'])
    gui = LaunchConfiguration('gui')
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    models = os.path.join(get_package_share_directory('scara_sort_sim'), 'models')
    resource_path = os.pathsep.join(
        p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), models] if p)

    # Run xacro to turn the .xacro file into plain URDF text
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),

        # The world includes the bins as model://sort_bin, from this package's models folder.
        # (Built without empty entries: an empty entry makes Gazebo search the root folder.)
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),

        # 1. Gazebo with our world (-r = start running, not paused), with or without its window
        IncludeLaunchDescription(
            gz_launch,
            launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            gz_launch,
            launch_arguments={'gz_args': ['-r -s --headless-rendering ', world],
                              'on_exit_shutdown': 'true'}.items(),
            condition=UnlessCondition(gui),
        ),

        # 2. Publish the robot model and its frames (TF), from /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),

        # 3. Put the arm into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'scara'],
        ),

        # 4. Connect Gazebo topics to ROS 2 (list in config/bridge.yaml)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        ),

        # 5. RViz to see the arm and its frames
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
