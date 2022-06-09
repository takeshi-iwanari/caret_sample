import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
  launch_args = [
  ]

  sub_launches = [
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare("caret_sample"),
          "launch",
          "caret.launch.py"
        ])
      ])
    ),
  ]

  nodes = [
      Node(
          package="caret_sample",
          executable="sample_0",
          output="screen",
          emulate_tty=True,
      ),
  ]
  
  # todo: make sure that trace starts before sample node

  return LaunchDescription(launch_args + sub_launches + nodes)

