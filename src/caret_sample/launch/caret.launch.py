
import launch
from tracetools_launch.action import Trace

import sys
import datetime
from distutils.util import strtobool


def generate_launch_description():
  caret_session = ""
  caret_event = ["ros2*"]
  caret_light = True

  for arg in sys.argv:
    if arg.startswith("caret_session:="):
      caret_session = arg.split(":=")[1]
    elif arg.startswith("caret_light:="):
      try:
        caret_light = strtobool(arg.split(":=")[1]) # 0 or 1
      except:
        print("Invalid arguments 'caret_light'.")
        print("Start tracing with 'ros2*'.")

  if caret_light:
    caret_event = [ "ros2:*callback*",
                "ros2:dispatch*",
                "ros2:rclcpp*",
                "ros2:rcl_*init",
                "*callback_group*",
                "ros2_caret:*callback*",
                "ros2_caret:dispatch*",
                "ros2_caret:rclcpp*",
                "ros2_caret:rcl_*init",
                "ros2_caret:rmw*",
                "ros2_caret:*executor",
                "ros2_caret:dds_bind*",
    ]

  if caret_session == "":
    dt_now = datetime.datetime.now()
    # caret_session = "caret_sample_" + dt_now.strftime("%Y%m%d-%H%M%S")
    caret_session = "caret_sample"

  return launch.LaunchDescription([
      Trace(
          session_name=caret_session,
          events_kernel=[],
          events_ust=caret_event
      )
  ])

if __name__ == "__main__":
  generate_launch_description()
