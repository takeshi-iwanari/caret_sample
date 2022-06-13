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
  app_name = LaunchConfiguration("app_name")
  launch_args = [
    DeclareLaunchArgument(
      "app_name",
      default_value="sample_straight"
    ),
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
          executable=app_name,
          output="screen",
          emulate_tty=True,
      ),
  ]
  
  # todo: make sure that trace starts before sample node

  return LaunchDescription(launch_args + sub_launches + nodes)

