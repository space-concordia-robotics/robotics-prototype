# building from ros:kinetic image
FROM ros:kinetic 

# create a user for this image with username 'scuser'
RUN useradd scuser \
# disabling password for sudo (required to work with this image)
&& echo 'scuser ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

# switching to new user
USER scuser

ENV USER=scuser

# set workdir
WORKDIR home/scuser/Programming/robotics-prototype/ 

# move root bashrc to home
RUN sudo mv /root/.bashrc ~/ \
&& sudo apt-get update \
&& sudo apt-get install -y python-pip \
virtualenv \
git \
vim \
# vim is mandatory 
software-properties-common \
&& sudo apt-get update \
&& sudo add-apt-repository ppa:deadsnakes/ppa \
&& sudo apt-get update \
&& sudo apt-get install -y python3.6 


# clone the repo
RUN sudo git clone --recursive https://github.com/space-concordia-robotics/robotics-prototype . ; exit 0

# creating venv
ENV VIRTUAL_ENV=venv
RUN sudo virtualenv --python=/usr/bin/python3.6 $VIRTUAL_ENV  
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

RUN python --version \
&& sudo chown -R scuser /home/scuser/ \
&& python -m pip install --upgrade pip \
&& python -m pip install -r requirements.txt -r requirements-dev.txt \
&& python setup.py develop \
&& pytest \
&& sudo apt-get install -y ros-kinetic-rosbridge-suite \
ros-kinetic-cv-camera \
ros-kinetic-web-video-server \
&& rosdep update \
&& /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd ~/Programming/robotics-prototype/robot/rospackages ; rosdep install --from-paths src/ --ignore-src -r -y ; catkin_make' \
&& echo '\n#ros aliases\nsource /opt/ros/kinetic/setup.bash\n#competition mode\n#export ROS_MASTER_URI=http://172.16.1.30:11311\n#export ROS_HOSTNAME=$USER\n. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash\n. ~/Programming/robotics-prototype/venv/bin/activate\nsource ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases' >> ~/.bashrc \
&& robot/basestation/env.sh >| robot/basestation/static/js/env.js 
# && cp commit-message-hook.sh .git/hooks/prepare-commit-msg && cp branch_name_verification_hook.py .git/hooks/post-checkout 
docker run -p 5000:5000 -p 9090:9090 -it --rm --name my-running-app -v scvolume:/home/scuser/Programming/robotics-prototype my-ros-k-app
