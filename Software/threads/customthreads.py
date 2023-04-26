import sys
sys.path.append("/home/mcqueen/mcqueen/Software/libs/encoder")
sys.path.append("/home/mcqueen/mcqueen/Software/libs/tis")

import time
import board
import logging
import traceback
import os
import csv
import copy

from  datetime import datetime
from threading import Thread, Event
from collections import deque
from busio import I2C
from adafruit_bno055 import BNO055_I2C
from pyjoystick.sdl2 import run_event_loop
from jtop import jtop

from encoder import Encoder

class IMUThread(Thread):
    def __init__(self, pipe, stop_event, init_event, pause_event):
        super(IMUThread, self).__init__(name="IMUThread")
             
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event
        self.pause_event: Event = pause_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising I2C bus 2...")
            bus_i2c_2 = I2C(board.SCL_1, board.SDA_1)
            
            logging.info("Initialising IMU...")
            sensor_imu = BNO055_I2C(bus_i2c_2)
            
            self.init_event.set()
            
            logging.info("Starting IMU")
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    time.sleep(0.5)
                    continue
                
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
            traceback.print_exc()
            self.stop_event.set()
        finally:
            logging.info("Stopped IMU")

class EncoderThread(Thread):
    def __init__(self, pipe, stop_event, init_event, pause_event):
        super(EncoderThread, self).__init__(name="EncoderThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event
        self.pause_event: Event = pause_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Encoder...")
            sensor_encoder = Encoder(board.D22, board.D23)
            self.init_event.set()
            
            logging.info("Starting Encoder")
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    time.sleep(0.5)
                    continue
                
                self.pipe.append({
                    'time': datetime.now(),
                    'position': sensor_encoder.getValue()
                })
                time.sleep(0.1)
                
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
        finally:
            logging.info("Stopped Encoder")
            
class StatsThread(Thread):
    def __init__(self, pipe, stop_event, init_event, pause_event):
        super(StatsThread, self).__init__(name="StatsThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event
        self.pause_event: Event = pause_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Stats...")
            self.init_event.set()
            
            logging.info("Starting Stats")
            with jtop() as jetson:
                while not self.stop_event.is_set() and jetson.ok():
                    if self.pause_event.is_set():
                        time.sleep(0.5)
                        continue
                
                    self.pipe.append(jetson.stats)
                
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
        finally:
            logging.info("Stopped Stats")
                
class ControllerThread(Thread):
    def __init__(self, pipe, stop_event, init_event):
        super(ControllerThread, self).__init__(name="ControllerThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Controller...")
            self.init_event.set()
            
            logging.info("Starting Controller")
            run_event_loop(self.__controller_add, self.__controller_remove, self.__controller_process, alive=self.__controller_alive)
            
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
            
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
    def __init__(self, pipe, stop_event, init_event, pause_event):
        super(ImageProcessingThread, self).__init__(name="ImageProcessingThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event
        self.pause_event: Event = pause_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Image Processing...")
            self.init_event.set()
            
            logging.info("Starting Image Processing")
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    time.sleep(0.5)
                    continue
                
                # Add image processing algorithm here
                time.sleep(1)
                
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
        finally:
            logging.info("Stopped Image Processing")

class DataCollectionThread(Thread):
    def __init__(self, pipe, stop_event, init_event, pause_event, pipes):
        super(DataCollectionThread, self).__init__(name="DataCollectionThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event
        self.pause_event: Event = pause_event
        self.pipes : dict(deque) = pipes

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Data Collection...")
            path_base = "/media/mcqueen/MCQUEEN"
            path_data = "/data/" + datetime.now().strftime("%d%m%Y %H%M%S")
            self.path = path_base + path_data
            if not os.path.exists(path_base):
                logging.warning("USB drive not detected, data collection disabled!")
                return
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            
            self.init_event.set()
            
            logging.info("Starting Data Collection")
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    time.sleep(0.5)
                    continue
                
                self.__save()
                time.sleep(10)

            # Save remaining pipe contents on quit
            self.__save()
                
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
        finally:
            logging.info("Stopped Data Collection")

    def __save(self):
        # Make local copy to prevent locked pipes
        pipes = copy.deepcopy(self.pipes)
        # Clear pipes
        for pipe in self.pipes.values():
            pipe.clear()
        # Save local copy to file
        for name, pipe in pipes.items():
            if len(pipe) > 0:
                filename = name + ".csv"
                with open(self.path + "/" + filename, 'w') as file:
                    writer = csv.writer(file, delimiter="|")
                    writer.writerow(list(pipe[0].keys()))
                    for item in pipe:
                        writer.writerow(item.values())