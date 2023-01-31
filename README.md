# ros-seminar

## Setting the environment
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$PWD/worlds
```

## Scenario launching
```bash
gazebo worlds/simple_circuit.world
```

<!-- ## Launching
```bash
roslaunch launch/simple_circuit.launch
``` -->

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