import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    args_record_topics = DeclareLaunchArgument('record_topics', default_value='False', choices=['True', 'False'])  
    walker_node = Node(
            package='turtlebot3_walker_ros2',
            executable='walker',
            name='walker'
        )
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
                                      '/turtlebot3_world.launch.py'])
    )
    recorder_node = ExecuteProcess(
        condition = IfCondition(LaunchConfiguration('record_topics')),
        cmd = ['ros2', 'bag', 'record', '-o', 'walker_bag_topics_list', '-a' , '-x "/camera.+"'],
        shell=True
    )
    return LaunchDescription([
        gazebo_world,
        args_record_topics,
        walker_node,
        recorder_node
    ])