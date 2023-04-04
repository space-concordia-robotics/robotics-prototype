#! /bin/bash

sudo apt update
sudo apt upgrade -y
sudo apt-get install -y python3-pip python3-venv

python3 -m venv venv
source venv/bin/activate

pip install -U pip
pip install -r requirements.txt

python setup.py develop

sudo apt-get install snapd -y
sudo apt-get install zip unzip

