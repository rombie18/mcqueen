import time
from threading import Event, Thread
from collections import deque
from jtop import jtop
import os
import csv
import board
import time
import math
import logging

from simple_pid import PID
from busio import I2C
# from rotaryio import IncrementalEncoder
from adafruit_bno055 import BNO055_I2C
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as MOTOR

import pyjoystick
from pyjoystick.sdl2 import Key, Joystick, run_event_loop

import threading
from subprocess import call
import random
from datetime import datetime


class Mcqueen:
    def __init__(self):
        # Variables
        self.heading = 0
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s - %(message)s")

        # Busses
        print("Initialising busses...")
        bus_i2c_2 = I2C(board.SCL_1, board.SDA_1)

        # Sensors
        print("Initialising sensors...")
        self.sensor_imu = BNO055_I2C(bus_i2c_2)

        # Telemetry
        self.path = "data/" + datetime.now().strftime("%d%m%Y %H:%M:%S")
        if not os.path.exists(self.path):
            os.makedirs(self.path)

        # Threads
        print("Starting threads...")
        self.stop_event = Event()
        self.startThreads()

    def startThreads(self):

        def handle_produce_sensor_imu(self):
            return {
                'time': datetime.now(),
                'temperature': self.sensor_imu.temperature,
                'acceleration': self.sensor_imu.acceleration,
                'magnetic': self.sensor_imu.magnetic,
                'gyro': self.sensor_imu.gyro,
                'euler': self.sensor_imu.euler,
                'quaternion': self.sensor_imu.quaternion,
                'linear_acceleration': self.sensor_imu.linear_acceleration,
                'gravity': self.sensor_imu.gravity
            }
            
        def handle_produce_sensor_stats(self):
            with jtop() as jetson:
                return jetson.stats

        def handle_read_sensor_imu(self, item):
            self.heading = item["euler"][0]

        def handle_consume_sensor_imu(self, items):
            filename = "imu.csv"
            with open(self.path + "/" + filename, 'w') as file:
                writer = csv.writer(file, delimiter="|")
                writer.writerow(list(items[0].keys()))
                for item in items:
                    writer.writerow(item.values())
                    
        def handle_consume_sensor_stats(self, items):
            filename = "stats.csv"
            with open(self.path + "/" + filename, 'w') as file:
                writer = csv.writer(file, delimiter="|")
                writer.writerow(list(items[0].keys()))
                for item in items:
                    writer.writerow(item.values())

        pipe_sensor_imu = deque()
        pipe_sensor_stats = deque()
        
        thread_producer_sensor_imu = ProducerThread(
            pipe_sensor_imu, self.stop_event, 100, handle_produce_sensor_imu, self)
        thread_producer_sensor_stats = ProducerThread(
            pipe_sensor_stats, self.stop_event, 1, handle_produce_sensor_stats, self)
        
        thread_reader_sensor_imu = ReaderThread(
            pipe_sensor_imu, self.stop_event, 10, handle_read_sensor_imu, self)
        
        thread_consumer_sensor_imu = ConsumerThread(
            pipe_sensor_imu, self.stop_event, 0.1, handle_consume_sensor_imu, self)
        thread_consumer_sensor_stats = ConsumerThread(
            pipe_sensor_stats, self.stop_event, 0.1, handle_consume_sensor_stats, self)

        thread_producer_sensor_imu.start()
        thread_producer_sensor_stats.start()

        thread_reader_sensor_imu.start()
        
        thread_consumer_sensor_imu.start()
        thread_consumer_sensor_stats.start()


class ProducerThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_produce, argument):
        super(ProducerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_produce = handle_produce
        self.argument = argument

    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            value = self.handle_produce(self.argument)
            self.pipe.appendleft(value)
            logging.debug("Produced: %s -> %s", str(value), str(self.pipe))
            time.sleep(self.period - ((time.time() - starttime) % self.period))


class ReaderThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_read, argument):
        super(ReaderThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_read = handle_read
        self.argument = argument

    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            if len(self.pipe) > 0:
                self.handle_read(self.argument, self.pipe[0])
                logging.debug("Read: %s -> %s",
                              str(self.pipe[0]), str(self.pipe))
            else:
                logging.debug("Read: Empty pipe")
            time.sleep(self.period - ((time.time() - starttime) % self.period))


class ConsumerThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_consume, argument):
        super(ConsumerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_consume = handle_consume
        self.argument = argument

    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            if len(self.pipe) > 10:
                values = []
                while len(self.pipe) > 10:
                    values.append(self.pipe.pop())
                self.handle_consume(self.argument, values)
                logging.debug("Consumed: %s -> %s",
                              str(values), str(self.pipe))
                time.sleep(self.period -
                           ((time.time() - starttime) % self.period))
        values = []
        while len(self.pipe) > 0:
            values.append(self.pipe.pop())
        self.handle_consume(self.argument, values)
        logging.debug("Consumed: %s -> %s", str(values), str(self.pipe))


mcQueen = Mcqueen()
