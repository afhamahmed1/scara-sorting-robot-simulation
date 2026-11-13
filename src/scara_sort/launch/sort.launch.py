"""Start the whole sorting cell: simulation, suction cup, spawner, detector, sorter, referee.

    ros2 launch scara_sort sort.launch.py
    ros2 launch scara_sort sort.launch.py rules:=config/sort_by_shape.yaml
    ros2 launch scara_sort sort.launch.py count:=10 seed:=2 gui:=false
    ros2 launch scara_sort sort.launch.py mode:=move speed:=0.10      # picking on the move

The suction node warms up its slots first; the spawner and the sorter wait for it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rules = LaunchConfiguration('rules')
    count = ParameterValue(LaunchConfiguration('count'), value_type=int)
    seed = ParameterValue(LaunchConfiguration('seed'), value_type=int)
    speed = ParameterValue(LaunchConfiguration('speed'), value_type=float)

    def node(executable, parameters=None):
        return Node(package='scara_sort', executable=executable, output='screen',
                    parameters=[{'use_sim_time': True, **(parameters or {})}])

    return LaunchDescription([
        DeclareLaunchArgument('rules', default_value='config/sort_by_colour.yaml',
                              description='Rules file: which kind of item goes into which bin'),
        DeclareLaunchArgument('count', default_value='30', description='Items in the run, 0 = endless'),
        DeclareLaunchArgument('seed', default_value='1', description='Same seed, same items'),
        DeclareLaunchArgument('speed', default_value='0.05', description='Belt speed, m/s'),
        DeclareLaunchArgument('mode', default_value='stop',
                              description='stop = stop the belt for each pick, move = never stop it'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('scara_sort_sim'), 'launch', 'sim.launch.py']),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        node('suction'),
        node('item_spawner', {'count': count, 'seed': seed}),
        node('item_detector'),
        node('sorter', {'rules': rules, 'speed': speed, 'mode': LaunchConfiguration('mode')}),
        node('referee', {'rules': rules, 'count': count}),
    ])
