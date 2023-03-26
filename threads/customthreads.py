import time
import board
import logging
from  datetime import datetime
from threading import Thread, Event
from collections import deque
from busio import I2C
from adafruit_bno055 import BNO055_I2C
from encoder import Encoder


class IMUThread(Thread):
    def __init__(self, pipe, stop_event):
        super(IMUThread, self).__init__(name="IMUThread")
        logging.getLogger()
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        
        logging.info("Initialising IMU...")
        bus_i2c = I2C(board.SCL_1, board.SDA_1)
        self.sensor_imu = BNO055_I2C(bus_i2c)

    def run(self):
        logging.info("Starting IMU")
        while not self.stop_event.is_set():
            self.pipe.append({
                'time': datetime.now(),
                'temperature': self.sensor_imu.temperature,
                'acceleration': self.sensor_imu.acceleration,
                'magnetic': self.sensor_imu.magnetic,
                'gyro': self.sensor_imu.gyro,
                'euler': self.sensor_imu.euler,
                'quaternion': self.sensor_imu.quaternion,
                'linear_acceleration': self.sensor_imu.linear_acceleration,
                'gravity': self.sensor_imu.gravity
            })
            time.sleep(0.1)
            

class EncoderThread(Thread):
    def __init__(self, pipe, stop_event):
        super(EncoderThread, self).__init__(name="EncoderThread")
        logging.getLogger()
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        
        logging.info("Initialising Encoder...")
        self.sensor_encoder = Encoder(board.D17, board.D18)

    def run(self):
        logging.info("Starting Encoder")
        while not self.stop_event.is_set():
            self.pipe.append({
                'time': datetime.now(),
                'position': self.sensor_encoder.getValue()
            })
            time.sleep(0.1)