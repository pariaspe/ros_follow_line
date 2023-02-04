FROM ros:noetic

RUN apt-get update && apt-get install -y \
    wget

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update \
    && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins

ENV USERNAME user

RUN groupadd -r $USERNAME && \
    useradd -r -g $USERNAME -m -s /bin/bash $USERNAME
RUN usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME
USER $USERNAME
WORKDIR /home/${USERNAME}

RUN mkdir noetic_ws
COPY models ./noetic_ws/models/
COPY worlds ./noetic_ws/worlds/
COPY launch ./noetic_ws/launch/
COPY docker/entrypoint.bash .

ENV GAZEBO_MODEL_PATH /home/${USERNAME}/noetic_ws/models:/home/${USERNAME}/noetic_ws/worlds
ENV GAZEBO_RESOURCE_PATH /home/${USERNAME}/noetic_ws/launch:/home/${USERNAME}/noetic_ws/worlds
ENV DISPLAY :0

ENTRYPOINT ["./entrypoint.bash"]
CMD ["bash"]