FROM ros:melodic-ros-core

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y ros-melodic-rosbridge-suite \
    && useradd --create-home --groups sudo --shell /bin/bash spaceuser \
    && echo -e 'spaceuser\nspaceuser' | passwd spaceuser \
    && mkdir -p /home/spaceuser/Programming/robotics-prototype


WORKDIR /home/spaceuser/Programming/robotics-prototype
COPY requirements.txt requirements-dev.txt setup.py docker/rosbridge/entrypoint.sh ./

RUN apt-get update && apt-get install -y python3-pip \
    && pip3 install -r requirements.txt -r requirements-dev.txt \
    && pip3 install rosdep \
    && python3 setup.py develop \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -y && apt-get upgrade -y \
    && apt-get install -y nano vim

COPY . .
WORKDIR /home/spaceuser/Programming/robotics-prototype/robot/rospackages
RUN rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install --from-paths src/ --ignore-src -r -y \
    && source /opt/ros/$ROS_DISTRO/setup.sh \
    && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && chown -R spaceuser:spaceuser /home/spaceuser/Programming \
    && chmod u+x /home/spaceuser/Programming/robotics-prototype/entrypoint.sh

WORKDIR /home/spaceuser/Programming/robotics-prototype

USER spaceuser

# $USER env variable used in some of the scripts
ENV USER spaceuser 
WORKDIR /home/spaceuser/Programming/robotics-prototype

RUN echo "source /opt/ros/melodic/setup.bash" >> /home/spaceuser/.bashrc \
    && echo "source /home/spaceuser/Programming/robotics-prototype/robot/rospackages/devel/setup.bash" >> /home/spaceuser/.bashrc \
    && echo "source /home/spaceuser/Programming/robotics-prototype/robot/basestation/config/.bash_aliases" >> /home/spaceuser/.bashrc \
    && echo "source /home/spaceuser/Programming/robotics-prototype/robot/rover/config/.bash_aliases" >> /home/spaceuser/.bashrc

ENTRYPOINT ["./entrypoint.sh"]
