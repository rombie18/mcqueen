import board
import time
import math
import json

from simple_pid import PID
from busio import I2C
# from rotaryio import IncrementalEncoder
from adafruit_bno055 import BNO055_I2C
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as MOTOR

import pyjoystick
from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from encoder import Encoder

import threading
from subprocess import call
import sys
import cv2
import numpy as np
sys.path.append("python-common")

import TIS

# Image processing
print("Initialising image processing...")
Tis = TIS.TIS()
Tis.open_device("02320237", 1280, 720, "60/1", TIS.SinkFormats.BGRA, True)
Tis.start_pipeline()

with open('camera_properties.json', 'r') as file:
    data = json.load(file)
    for key, item in data.items():
        try:
            Tis.set_property(key, item)
        except Exception as error:
            pass

while True:
    time.sleep(0.1)