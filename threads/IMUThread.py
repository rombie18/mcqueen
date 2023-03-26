import time
import datetime
import logging
import board
from threading import Thread, Event
from collections import deque
from busio import I2C
from adafruit_bno055 import BNO055_I2C

class IMUThread(Thread):
    def __init__(self, pipe, stop_event):
        super(IMUThread, self).__init__()
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        
        logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
        
        bus_i2c = I2C(board.SCL_1, board.SDA_1)
        self.sensor_imu = BNO055_I2C(bus_i2c)

    def run(self):
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