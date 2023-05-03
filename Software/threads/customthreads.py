import sys
sys.path.append("/home/mcqueen/mcqueen/Software/libs/encoder")
sys.path.append("/home/mcqueen/mcqueen/Software/libs/tis")
sys.path.append("/home/mcqueen/mcqueen/Software/libs/lane_detection")

import time
import board
import logging
import traceback
import os
import csv
import copy
import cv2
import json
import numpy
import math

from  datetime import datetime
from threading import Thread, Event
from collections import deque
from busio import I2C
from adafruit_bno055 import BNO055_I2C
from pyjoystick.sdl2 import run_event_loop
from jtop import jtop

from encoder import Encoder
from lane import Lane
from tis import TIS, SinkFormats

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
            
            logging.info("IMU initialised.")
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
            
            logging.info("Encoder initialised.")
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    time.sleep(0.5)
                    continue
                
                self.pipe.append({
                    'time': datetime.now(),
                    'time_epoch': time.time(),
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

            with jtop() as jetson:
                self.init_event.set()
                logging.info("Stats initialised.")
            
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
            
            run_event_loop(self.__controller_add, self.__controller_remove, self.__controller_process, alive=self.__controller_alive)
            
            self.init_event.set()
            logging.info("Controller initialised.")

            
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
            
    def __controller_add(self, joy):
        data = {
            'time': datetime.now(),
            'type': "add"
        }
        self.pipe.append(data)
        
    def __controller_remove(self, joy):
        data = {
            'time': datetime.now(),
            'type': "remove",
        }
        self.pipe.append(data)
        
    def __controller_process(self, key):
        data = {
            'time': datetime.now(),
            'type': "event",
            'key': key
        }
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
            camera = TIS()
            camera.open_device("02320237", 1280, 720, "60/1", SinkFormats.BGRA, False)

            with open('/home/mcqueen/mcqueen/Software/main/camera_properties.json', 'r') as file:
                data = json.load(file)
                for key, item in data.items():
                    camera.set_property(key, item)

            try:
                camera.set_property("TriggerMode","Off")
            except Exception as error:
                print(error)

            camera.start_pipeline()
            self.init_event.set()
            
            logging.info("Image Processing initialised.")
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    time.sleep(0.5)
                    continue
                
                if camera.snap_image(1):                                        
                    # Load a frame (or image)
                    original_frame = camera.get_image()
                    
                    # Discard unwanted part of image
                    original_height, original_width, original_channels = original_frame.shape
                    cropped_frame = original_frame[280:original_height, 0:original_width]
                    
                    # Create a Lane object
                    lane_obj = Lane(orig_frame=cropped_frame)
                    
                    # Perform thresholding to isolate lane lines
                    lane_line_markings = lane_obj.get_line_markings()
                    # If there are less than 50 'lane' pixels detected, skip calculations
                    #if numpy.sum(lane_line_markings == 255) < 50:
                    #    continue
                    
                    # Plot the region of interest on the image
                    lane_obj.plot_roi(plot=False)
                    
                    # Perform the perspective transform to generate a bird's eye view
                    # If Plot == True, show image with new region of interest
                    warped_frame = lane_obj.perspective_transform(plot=False)
                    
                    # Generate the image histogram to serve as a starting point
                    # for finding lane line pixels
                    histogram = lane_obj.calculate_histogram(plot=False)  
                        
                    # Find lane line pixels using the sliding window method 
                    left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
                        plot=False)
                    
                    # Fill in the lane line
                    lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
                        
                    # Overlay lines on the original frame
                    frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
                    
                    # Calculate lane line curvature (left and right lane lines)
                    lane_obj.calculate_curvature(print_to_terminal=True)
                    
                    # Calculate center offset                                                                 
                    lane_obj.calculate_car_position(print_to_terminal=True)

                    # Calculate turning angle
                    wheel_base = 0.3
                    curvem = (lane_obj.left_curvem + lane_obj.right_curvem) / 2
                    angle = math.degrees(math.atan(wheel_base / curvem))

                    print(angle)

                    # Append result to pipe
                    self.pipe.append({
                        'time': time.time(),
                        'center_offset': lane_obj.center_offset,
                        'left_curvem': lane_obj.left_curvem,
                        'right_curvem': lane_obj.right_curvem,
                        'curvem': curvem,
                        'angle': angle
                    })
                                
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
                self.init_event.set()
                return
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            
            self.init_event.set()
            
            logging.info("Data Collection initialised.")
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