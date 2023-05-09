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
import pyjoystick
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
                    try:                                  
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
                        
                        # If detected to little pixels, skip futher calculations
                        detected_pixels = numpy.sum(warped_frame == 255)
                        if detected_pixels < 3000:
                            logging.debug("Only {} valid pixels, skipping this frame".format(detected_pixels))
                            continue
                        
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
                        lane_obj.calculate_curvature(print_to_terminal=False)
                        
                        # Calculate center offset                                                                 
                        lane_obj.calculate_car_position(print_to_terminal=False)

                        # Calculate turning angle
                        wheel_base = 0.3
                        curvem = (lane_obj.left_curvem + lane_obj.right_curvem) / 2
                        angle = math.degrees(math.atan(wheel_base / curvem)) * 6 + 90

                        logging.debug("Curve radius left: " + str(angle))
                        logging.debug("Curve radius right: " + str(angle))
                        logging.debug("Steering angle: " + str(angle))

                        # Append result to pipe
                        data = {
                            'time': datetime.now(),
                            'center_offset': lane_obj.center_offset,
                            'left_curvem': lane_obj.left_curvem,
                            'right_curvem': lane_obj.right_curvem,
                            'curvem': curvem,
                            'angle': angle
                        }
                        self.pipe.append(data)

                    except:
                        logging.warn("Unable to process image")
                                
        except Exception as exception:
            logging.error(exception)
            traceback.print_exc()
            self.stop_event.set()
        finally:
            logging.info("Stopped Image Processing")

class LowImageProcessingThread(Thread):
    def __init__(self, pipe, stop_event, init_event, pause_event):
        super(LowImageProcessingThread, self).__init__(name="LowImageProcessingThread")
        
        self.pipe: deque = pipe
        self.stop_event: Event = stop_event
        self.init_event: Event = init_event
        self.pause_event: Event = pause_event

    def run(self):
        try:
            logging.getLogger()
            
            logging.info("Initialising Low Image Processing...")
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
                    video = camera.get_image()

                    X_lb=0
                    Y_lb=0
                    X_rb=0
                    Y_rb=0

                    h1= 0
                    h2= 0

                    #Specifying upper and lower ranges of color to detect in hsv format
                    lower = numpy.array([137, 0 ,17 ])
                    upper = numpy.array([180, 255 ,255 ])
                    lower_2 = numpy.array([0, 0 ,0 ])
                    upper_2 = numpy.array([0, 0 ,0 ])

                    hight, width, _ = video.shape #Get resolution

                    video_cropped = video[307:hight, 0:width]

                    video_hsv = cv2.cvtColor(video_cropped, cv2.COLOR_BGR2HSV) #Converting BGR image to HSV format

                    mask_video_1 = cv2.inRange(video_hsv, lower, upper) # Masking the image to find our color
                    mask_video_2 = cv2.inRange(video_hsv, lower_2, upper_2)

                    mask_video = mask_video_1 | mask_video_2

                    mask_leftBand = mask_video[0:hight, int(200):int(250)] #Crop right band
                    mask_rightBand = mask_video[0:hight, int(width-250):int(width-200)] #crop left band

                    mask_contours_leftBand, hierarchy = cv2.findContours(mask_leftBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (leftband)
                    mask_contours_rightBand, hierarchy = cv2.findContours(mask_rightBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (rightband)

                    # Finding position of all contours of the bands
                    if len(mask_contours_leftBand or mask_contours_rightBand) != 0:
                        for i_lb, k_rb in zip(mask_contours_leftBand, mask_contours_rightBand):
                            if cv2.contourArea(i_lb) > 500:
                                x1, y1, w1, h1 = cv2.boundingRect(i_lb)
                                X_lb = int(x1+(w1/2)) #calculate center of rectangle
                                Y_lb = int(y1+(h1/2))
                                cv2.rectangle(video_cropped, (x1+200, y1), (x1 + w1+200, y1 + h1), (0, 0, 255), 3) #drawing rectangle
                            
                            if cv2.contourArea(k_rb) > 500:
                                x2, y2, w2, h2 = cv2.boundingRect(k_rb)
                                X_rb = int(width-(x2+(w2/2))) #calculate center of rectangle
                                Y_rb = int(y2+(h2/2))
                                cv2.rectangle(video_cropped, ((width-x2-250), y2), (((width-x2-250) + w2), (y2 + h2)), (255, 0, 0), 3) #drawing rectangle (formula needed, otherwise it will add it on the left side)

                            
                        diff = 1
                        if h2 != 0:
                            diff = h1/h2
                        if (diff > 0.9) and (diff < 1.1):
                            angle = 90
                            print("Go straight")
                        if diff > 1.1: #h1>h2
                            norm = 1-(h1-h2)/h1
                            angle = 90*norm
                            print("Go right, with angle:", angle)
                        if diff < 0.9: #h1<h2
                            norm = ((h2-h1)/h2)
                            angle = 90+90*norm
                            print("Go left, with angle:", angle)
                        
                        # Append result to pipe
                        data = {
                            'time': datetime.now(),
                            'angle': angle
                        }
                        self.pipe.append(data)
                                
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
        pipes = {}
        for name, pipe in self.pipes.items():
            pipes[name] = pipe.copy()
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