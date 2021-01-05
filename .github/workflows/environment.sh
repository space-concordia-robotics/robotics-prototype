ROS_VERSION=melodic



sudo apt update
sudo apt upgrade -y
sudo apt install python3.6-venv git python3-pip net-tools -y

git clone --recursive -b gh-actions-493 https://github.com/space-concordia-robotics/robotics-prototype .
cd robotics-prototype

python3.6 -m venv venv
source venv/bin/activate

pip install -U pip
pip install -r requirements.txt

python setup.py develop
