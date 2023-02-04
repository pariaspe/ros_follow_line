# Docker instruction

## ROS master and gzserver on docker, gzclient on host
```bash
docker run -it -p 11345:11345 --device /dev/dri -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <tag> 
gzserver --verbose
```

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11345 gzclient
```