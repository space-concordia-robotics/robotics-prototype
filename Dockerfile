FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN useradd --create-home --shell /bin/bash spaceuser \
    && mkdir /home/spaceuser/robotics-prototype

WORKDIR /home/spaceuser/robotics-prototype
COPY requirements.txt requirements-dev.txt setup.py ./


RUN apt-get update && apt-get install -y python3-pip \
    && pip3 install -r requirements.txt -r requirements-dev.txt \
    && python3 setup.py develop 

RUN apt-get install -y ros-$ROS_DISTRO-cv-camera \
    && apt-get install -y ros-$ROS_DISTRO-web-video-server

COPY . .

WORKDIR /home/spaceuser/robotics-prototype/robot/rospackages
RUN rosdep install --from-paths src/ --ignore-src -r -y \
    && source /opt/ros/$ROS_DISTRO/setup.sh \
    && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && chown -R spaceuser:spaceuser /home/spaceuser/robotics-prototype

WORKDIR /home/spaceuser/robotics-prototype
USER spaceuser
RUN chmod u+x ./docker/prototype/entrypoint.sh
ENTRYPOINT ["./docker/prototype/entrypoint.sh"]
