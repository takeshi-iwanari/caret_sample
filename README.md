```sh
# 1. Install CARET
## Follow the instruction: https://tier4.github.io/CARET_doc/latest/

# 2. Create my ROS2 project
mkdir -p ~/caret_sample/src
cd ~/caret_sample/src
ros2 pkg create caret_sample --build-type ament_cmake --dependencies rclcpp rclcpp_components std_msgs
cd ..
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO

# 3. Write code

# 4. Build
source ~/ros2_caret_ws/install/local_setup.bash
colcon build --symlink-install

# 5. Start LTTng session (on another terminal)
## Don't need this, because trace automatically starts in launch file
# source ~/ros2_caret_ws/install/local_setup.bash
# ros2 trace -s test_caret -k -u "ros2*"

# 6. Run (on another terminal)
cd ~/caret_sample
source ~/ros2_caret_ws/install/local_setup.bash
source ./install/local_setup.bash
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"

ros2 launch caret_sample sample.launch.py app_name:=sample_0


# 7. Analyze
babeltrace ~/.ros/tracing/test_caret/ | cut -d' ' -f 4 | sort -u
## Open tutorial.ipynb in jupyter-lab or VSCode
```