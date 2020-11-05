FROM ros:melodic-ros-core

SHELL ["/bin/bash", "-c"]

# $USER env variable used in some of the scripts
ENV USER spaceuser 

RUN apt-get update && apt-get install -y ros-melodic-rosbridge-suite \
    && useradd --create-home --groups sudo --shell /bin/bash spaceuser \
    && echo -e 'spaceuser\nspaceuser' | passwd spaceuser \
    && mkdir -p /home/spaceuser/Programming/robotics-prototype \
    && echo "source /opt/ros/melodic/setup.bash" >> /home/spaceuser/.bashrc \
    && echo "source /home/spaceuser/Programming/robotics-prototype/robot/rospackages/devel/setup.bash" >> /home/spaceuser/.bashrc \
    && echo "source /home/spaceuser/Programming/robotics-prototype/robot/basestation/config/.bash_aliases" >> /home/spaceuser/.bashrc \
    && echo "source /home/spaceuser/Programming/robotics-prototype/robot/rover/config/.bash_aliases" >> /home/spaceuser/.bashrc 


WORKDIR /home/spaceuser/Programming/robotics-prototype
COPY requirements.txt requirements-dev.txt setup.py docker/rosbridge/entrypoint.sh ./

RUN apt-get update && apt-get install -y python3-pip \
    && pip3 install -r requirements.txt -r requirements-dev.txt \
    && pip3 install rosdep \
    && python3 setup.py develop \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -y && apt-get upgrade -y \
    && apt-get install -y nano vim wget curl libfontconfig libx11-6 libxft2

COPY install_arduino_teensyduino.sh .
USER spaceuser
RUN echo 'spaceuser' | sudo -S /home/spaceuser/Programming/robotics-prototype/install_arduino_teensyduino.sh 

USER root
COPY robot/rospackages ./robot/rospackages
RUN rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install --from-paths robot/rospackages/src/ --ignore-src -r -y 

USER spaceuser
COPY robot/rover ./robot/rover
WORKDIR /home/spaceuser/Programming/robotics-prototype/robot/rover
RUN echo 'spaceuser' | sudo -S chown -R spaceuser:spaceuser /home/spaceuser \
    && tail -n1 /home/spaceuser/.bashrc > /home/spaceuser/.tmprc \
    && source /home/spaceuser/.tmprc \
    && cmake . \
    && make -j$(nproc) \
    && rm /home/spaceuser/.tmprc

USER root
WORKDIR /home/spaceuser/Programming/robotics-prototype

COPY . .

WORKDIR /home/spaceuser/Programming/robotics-prototype/robot/rospackages
RUN source /opt/ros/$ROS_DISTRO/setup.sh \
    && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && chown -R spaceuser:spaceuser /home/spaceuser/Programming \
    && chmod u+x /home/spaceuser/Programming/robotics-prototype/entrypoint.sh

USER spaceuser
WORKDIR /home/spaceuser/Programming/robotics-prototype

ENTRYPOINT ["./entrypoint.sh"]
