# Docker instruction

## Building
```bash
docker build -f docker/Dockerfile -t <tag> .
```

## ROS master and gzserver on docker, gzclient on host
```bash
docker run -it -p 11345:11345 --device /dev/dri -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <tag>
roslaunch ros_follow_line docker_simple_circuit.launch  # or just "gzserver --verbose"
```

On host machine:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/models
GAZEBO_MASTER_URI=http://127.0.0.1:11345 gzclient --verbose
```