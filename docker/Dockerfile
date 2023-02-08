FROM ros:noetic

RUN apt-get update && apt-get install -y \
    wget

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update \
    && apt-get install -y \
        gazebo11 \
        python3-catkin-tools \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-plugins

ENV USERNAME user

RUN groupadd -r $USERNAME && \
    useradd -r -g $USERNAME -m -s /bin/bash $USERNAME
RUN usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME
USER $USERNAME
WORKDIR /home/${USERNAME}

RUN mkdir -p noetic_ws/src/ros_follow_line
COPY . ./noetic_ws/src/ros_follow_line

ENV ROS_WORKSPACE=/home/${USERNAME}/noetic_ws
WORKDIR /home/${USERNAME}/noetic_ws
RUN catkin init
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; catkin build'

ENV DISPLAY :0

WORKDIR /home/${USERNAME}
COPY docker/entrypoint.bash .

ENTRYPOINT ["./entrypoint.bash"]
CMD ["bash"]