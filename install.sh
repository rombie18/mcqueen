#!/bin/bash

# Install generic libraries
apt update
apt install nano

# Install CircuitPython
apt install -y python3 git python3-pip
update-alternatives --install /usr/bin/python python $(which python2) 1
update-alternatives --install /usr/bin/python python $(which python3) 2
apt update
pip3 install --upgrade setuptools
pip3 install Jetson.GPIO
groupadd -f -r gpio
usermod -a -G gpio mcqueen
cp /home/mcqueen/.local/lib/python3.6/site-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger
pip3 install adafruit-blinka

# Install additional requirements
pip3 install -r requirements.txt

# Configure system service
chmod +x mcqueen-service.sh
cp mcqueen.service /etc/systemd/system/mcqueen.service
systemctl daemon-reload
systemctl enable mcqueen.service
systemctl start mcqueen.service