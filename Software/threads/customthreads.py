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
from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from adafruit_motor import servo as MOTOR
from simple_pid import PID
from adafruit_pca9685 import PCA9685


class IMUThread(Thread):
    def __init__(self, pipe, stop_event):
        super(IMUThread, self).__init__(name="IMUThread")        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising I2C bus 2...")
            bus_i2c_2 = I2C(board.SCL_1, board.SDA_1)
            
            logging.info("Initialising IMU...")
            sensor_imu = BNO055_I2C(bus_i2c_2)
            
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
                
        except Exception as exception:
            logging.error(exception)
            

class EncoderThread(Thread):
    def __init__(self, pipe, stop_event):
        super(EncoderThread, self).__init__(name="EncoderThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Encoder...")
            sensor_encoder = Encoder(board.D6, board.D12)
            
            logging.info("Starting Encoder")
            while not self.stop_event.is_set():
                self.pipe.append({
                    'time': datetime.now(),
                    'position': sensor_encoder.getValue()
                })
                time.sleep(0.1)
                
        except Exception as exception:
            logging.error(exception)
            
class StatsThread(Thread):
    def __init__(self, pipe, stop_event):
        super(StatsThread, self).__init__(name="StatsThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Starting Stats")
            with jtop() as jetson:
                while not self.stop_event.is_set() and jetson.ok():
                    self.pipe.append(jetson.stats)
                
        except Exception as exception:
            logging.error(exception)
                
class ControllerThread(Thread):
    def __init__(self, pipe, stop_event):
        super(ControllerThread, self).__init__(name="ControllerThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Starting Controller")
            run_event_loop(self.__controller_add, self.__controller_remove, self.__controller_process, alive=self.__controller_alive)
            
        except Exception as exception:
            logging.error(exception)
            
    def __controller_add(self, joy):
        data = {
            'time': datetime.now(),
            'type': "add"
        }
        data.update(vars(joy))
        self.pipe.append(data)
        
    def __controller_remove(self, joy):
        data = {
            'time': datetime.now(),
            'type': "remove"
        }
        data.update(vars(joy))
        self.pipe.append(data)
        
    def __controller_process(self, key):
        data = {
            'time': datetime.now(),
            'type': "event"
        }
        data.update(vars(key))
        self.pipe.append(data)
        
    def __controller_alive(self):
        return self.stop_event.is_set()
        
class ImageProcessingThread(Thread):
    def __init__(self, pipe, stop_event):
        super(ImageProcessingThread, self).__init__(name="ImageProcessingThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Image Processing...")
            pass
            
            logging.info("Starting Image Processing")
            while not self.stop_event.is_set():
                # Add image processing algorithm here
                time.sleep(1)
                
        except Exception as exception:
            logging.error(exception)