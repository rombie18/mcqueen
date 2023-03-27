import time
import board
import logging
from  datetime import datetime
from threading import Thread, Event
from collections import deque
from busio import I2C
from adafruit_bno055 import BNO055_I2C
from encoder import Encoder
from jtop import jtop


class IMUThread(Thread):
    def __init__(self, pipe, stop_event):
        super(IMUThread, self).__init__(name="IMUThread")        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        logging.getLogger()
        
        logging.info("Initialising IMU...")
        bus_i2c = I2C(board.SCL_1, board.SDA_1)
        sensor_imu = BNO055_I2C(bus_i2c)
        
        logging.info("Starting IMU")
        while not self.stop_event.is_set():
            self.pipe.append({
                'time': datetime.now(),
                'temperature': sensor_imu.temperature,
                'acceleration': sensor_imu.acceleration,
                'magnetic': sensor_imu.magnetic,
                'gyro': sensor_imu.gyro,
                'euler': sensor_imu.euler,
                'quaternion': sensor_imu.quaternion,
                'linear_acceleration': sensor_imu.linear_acceleration,
                'gravity': sensor_imu.gravity
            })
            time.sleep(0.1)
            

class EncoderThread(Thread):
    def __init__(self, pipe, stop_event):
        super(EncoderThread, self).__init__(name="EncoderThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        logging.getLogger()
        
        logging.info("Initialising Encoder...")
        sensor_encoder = Encoder(board.D17, board.D18)
        
        logging.info("Starting Encoder")
        while not self.stop_event.is_set():
            self.pipe.append({
                'time': datetime.now(),
                'position': sensor_encoder.getValue()
            })
            time.sleep(0.1)
            
class StatsThread(Thread):
    def __init__(self, pipe, stop_event):
        super(StatsThread, self).__init__(name="StatsThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        logging.getLogger()
        
        logging.info("Starting Stats")
        with jtop() as jetson:
            while not self.stop_event.is_set() and jetson.ok():
                self.pipe.append(jetson.stats)