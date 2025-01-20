### Why?
This is a basic project which shows a ROS2 node publishing controller data,
and another ROS2 node subscribing to that data.

### Build
- Set $ROS2 to the location of ROS2's files. Usually /opt/ros/{DISTRIBUTATION}/setup.zsh
- Set $CON_UTIL_DIR to the location the 'controller_utilities' library. Make sure it's built
- Set $ROS2_INPUT_MSGS to the location of the 'ros2_input_msgs' project. Make sure it's built
Note, all setup files are shell scripts. Change the extension to match what shell you're using. Default: zsh
```
source $ROS2/setup.zsh
source $ROS2_INPUT_MSGS/install/setup.zsh
colcon build --packages-select ros2_controller --cmake-args "-DCON_UTILS_DIR=$CON_UTILS_DIR"
```

### Run  
In one terminal:
```
source install/setup.zsh
source $ROS2_INPUT_MSGS/install/setup.zsh
ros2 run ros2_controller talker
```
  
In a second terminal:
```
source install/setup.zsh
source $ROS2_INPUT_MSGS/install/setup.zsh
ros2 run ros2_controller listener
```
