import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='listener_jointstates_cpp', executable='listener_jointstates_cpp', output='screen'),
    ])