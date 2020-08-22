ENV ROS_DISTRO=melodic
# building from ros:$ROS_DISTRO image
FROM ros:$ROS_DISTRO
SHELL ["/bin/bash", "-c"]
# create a user for this image with username '$USERNAME'
ARG USERNAME=spaceuser
RUN useradd $USERNAME \
# disabling password for sudo (required to work with this image)
&& echo $USERNAME 'ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
ENV USER=$USERNAME
# switching to new user
USER $USERNAME

ENV USER=$USERNAME

# set workdir
WORKDIR home/$USERNAME/Programming/robotics-prototype/ 

# move root bashrc to home
RUN sudo mv /root/.bashrc ~/ \
&& sudo apt-get update \
&& sudo apt-get install -y python-pip \
git \
vim \
net-tools \
software-properties-common \

# clone the repo
RUN sudo git clone --recursive https://github.com/space-concordia-robotics/robotics-prototype . ; exit 0

# creating venv
ENV VIRTUAL_ENV=venv
RUN sudo virtualenv --python=/usr/bin/python3.6 $VIRTUAL_ENV  
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

RUN python --version \
&& sudo chown -R $USERNAME /home/$USERNAME/ \
&& python -m pip install --upgrade pip \
&& python -m pip install -r requirements.txt -r requirements-dev.txt \
&& python setup.py develop \
&& pytest \
&& sudo apt-get install -y ros-$ROS_DISTRO-rosbridge-suite ros-$ROS_DISTRO-cv-camera ros-$ROS_DISTRO-web-video-server \
&& rosdep update \
&& . /opt/ros/$ROS_DISTRO/setup.bash \
&& cd ~/Programming/robotics-prototype/robot/rospackages \
&& rosdep install --from-paths src/ --ignore-src -r -y \
&& catkin_make \
&& echo '\n#ros aliases\n' >> ~/.bashrc \
&& echo 'source /opt/ros/$ROS_DISTRO/setup.bash\n' >> ~/.bashrc \
&& echo '#competition mode\n' >> ~/.bashrc \
&& echo '#export ROS_MASTER_URI=http://172.16.1.30:11311\n' >> ~/.bashrc \
&& echo '#export ROS_HOSTNAME=$USER\n' >> ~/.bashrc \
&& echo '. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash\n' >> ~/.bashrc \
&& echo '. ~/Programming/robotics-prototype/venv/bin/activate\n' >> ~/.bashrc \
&& echo '. ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases' >> ~/.bashrc
RUN . robot/basestation/env.sh >| robot/basestation/static/js/env.js 
RUN cd ~/Programming/robotics-prototype && \
cp commit_message_hook.py .git/hooks/prepare-commit-msg && cp branch_name_verification_hook.py .git/hooks/post-checkout 
