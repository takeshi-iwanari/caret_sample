import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  sub_launches = [
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare("autoware_dummy"),
          "launch",
          "caret.launch.py"
        ])
      ])
    ),
  ]

  nodes = [
      Node(
          package="autoware_dummy",
          executable="autoware_dummy",
          output="screen",
          emulate_tty=True,
      ),
  ]
  
  # todo: make sure that trace starts before sample node

  return LaunchDescription(sub_launches + nodes)

