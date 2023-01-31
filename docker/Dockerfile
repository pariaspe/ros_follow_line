FROM ros:noetic

RUN apt-get update && apt-get install -y \
    wget

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update \
    && apt-get install -y \
    gazebo11

ENV USERNAME user

RUN groupadd -r $USERNAME && \
    useradd -r -g $USERNAME -m -s /bin/bash $USERNAME
RUN usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME
USER $USERNAME
WORKDIR /home/${USERNAME}

RUN echo holaaaa

COPY models ./models/
COPY worlds ./worlds/

ENV GAZEBO_MODEL_PATH /home/${USERNAME}/models
ENV GAZEBO_RESOURCE_PATH /home/${USERNAME}/worlds

CMD ["bash"]