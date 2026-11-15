import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jps_planner')
    rviz_config = os.path.join(pkg_share, 'rviz', 'jps_planner.rviz')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/mcr/Workspace/RM/rosBasicsHomework2/mymap.yaml',
        description='Absolute path to the map yaml file.')

    jps_node = Node(
        package='jps_planner',
        executable='jps_planner_node',
        name='jps_planner_node',
        output='screen',
        parameters=[{'map_yaml': LaunchConfiguration('map_yaml')}])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    return LaunchDescription([map_yaml_arg, jps_node, rviz_node])
