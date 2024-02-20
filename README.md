# ROS Seminar

## Index
- [Overview](#overview)
- [First Steps](#first-steps)
- [ROS Interfaces](#ros-interfaces)
- [Troubleshooting](#troubleshooting)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## Overview

Programming a Formula 1 on a race track by following a red line in the center of the road with an RGB camera integrated into the F1 model.

<img src="docs/imgs/img_1.png" width="450"/>
<img src="docs/imgs/img_2.png" width="450"/>

### Goal

The goal of this exercise is to perform a PID reactive control capable of following the line painted on the racing circuit. The students have to implement a **ROS 2 node** that subscribes to the camera topic, processes the image and publishes the linear and angular velocities to the F1 model.

The student node should also be able to start and stop the game logic by calling the corresponding services. Besides, the controller constants should be easily configurable through the ROS 2 parameters and be given as arguments when launching the node.

As a summary, the student's node should have at least the following ROS 2 interfaces:
- 1 subscriber
- 1 publisher
- 2 services
- 1 parameter
- 1 launch file

### Auxiliary tools

There are two python modules that can be used to help the students to develop the solution:
- `color_filter.py`: This module contains a set of methods to process the image and extract the line position.
- `controller_pid.py`: This module contains the `PID` class that provides a method to calculate the control signal based on the error.

Both can be found in the `ros_follow_line` package and imported in your node.

```python
from ros_follow_line.controller_pid import PID
from ros_follow_line.color_filter import red_filter, get_centroid

# Example of usage
cv_bridge = CvBridge()
frame = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
img = red_filter(frame)
centroid = get_centroid(img)

##
controller = PID(k_p=1.0, k_d=1.0)
controller.setpoint = 0.0
output_vel = linear_controller.update(error)
```

## First Steps

### Set the workspace
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/pariaspe/ros_follow_line.git
cd ~/ros2_ws
colcon build
```

### Launching
```bash
ros2 launch ros_follow_line gazebo.launch.py
```

## ROS Interfaces
### Topics
| Topic name | ROS msg | Description |
| ---         | ---     | ---         |
| `/cam/image_raw`   | `sensor_msgs/msg/Image`      | F1 image publisher       |
| `/cam/camera_info` | `sensor_msgs/msg/CameraInfo` | F1 camera info publisher |
| `/odom`            | `nav_msgs/msg/Odometry`      | F1 odometry publisher    |
| `/cmd_vel`         | `geometry_msgs/msg/Twist`    | F1 commander publisher   |

### Services
| Service name | ROS msg | Description |
| ---         | ---     | ---         |
| `/game_logic/start` | `std_srvs/srv/Trigger` | Start Game Logic |
| `/game_logic/stop`  | `std_srvs/srv/Trigger` | Stop Game Logic  |


### Parameters
| Parameter name | Type | Description |
| ---         | ---     | ---         |
| `~k_p` | `double` | Proportional constant |
| `~k_i` | `double` | Integral constant |
| `~k_d` | `double` | Derivative constant |

## Troubleshooting

```bash
$  gazebo worlds/simple_circuit.world
[INFO] [1675081776.219252956] [gazebo_ros_node]: ROS was initialized without arguments.
[INFO] [1675081776.342421523] [object_controller]: Subscribed to [/cmd_vel]
[INFO] [1675081776.348074673] [object_controller]: Advertise odometry on [/odom]
[INFO] [1675081776.350898997] [object_controller]: Publishing odom transforms between [odom] and [base_footprint]
gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.
[INFO] [1675081778.089864820] [rclcpp]: signal_handler(signum=2)
```

[Fix](https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/):
```bash
 . /usr/share/gazebo/setup.sh 
```

## Acknowledgements
The idea of the exercise, the models and the worlds files are original from [RoboticsAcademy](https://github.com/JdeRobot/RoboticsAcademy) ([JdeRobot](https://github.com/JdeRobot)), where the author is an active maintainer.

## License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
