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
from enum import Enum

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
from encoder import Encoder


from pipedthread import ProducerThread, ReaderThread, ConsumerThread


class Mcqueen:
    def __init__(self):
        # Variables
        self.velocity = 0
        self.heading = 0
        self.stop = False
        self.pid_control = False
        self.set_heading = 0
        self.current_position = 0
        self.previous_position = 0
        
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s - %(message)s")

        # Busses
        print("Initialising busses...")
        bus_i2c_1 = I2C(board.SCL, board.SDA)
        bus_i2c_2 = I2C(board.SCL_1, board.SDA_1)

        # Other
        print("Initialising PWM driver...")
        pwmdriver = PCA9685(bus_i2c_1)
        pwmdriver.frequency = 50

        # Sensors
        print("Initialising sensors...")
        self.sensor_imu = BNO055_I2C(bus_i2c_2)
        # 162cm = 20 cycles op as encoder ==> per cycle = 8.1cm, 265 counts per cycle 
        self.sensor_encoder = Encoder(board.D17, board.D18, callback=self.handle_produce_sensor_encoder)

        # Actuators
        print("Initialising actuators...")
        # Steering servo rc car, T: 20.08ms, PW: (920µs to 2120µs) -> (1320µs to 1720µs) to limit dragging against axis
        # angle 0 =~ 15° right, angle 180 =~ 15° left
        self.actuator_servo = MOTOR.Servo(
            pwmdriver.channels[0], min_pulse=1220, max_pulse=1820)
        # Motor ESC rc car, T: 20.08ms, PW: (1000µs to 2000µs)
        self.actuator_motor = MOTOR.ContinuousServo(
            pwmdriver.channels[1], min_pulse=1000, max_pulse=2000)

        # Controllers
        print("Initialising controllers...")
        self.servo_pid = PID(3.5, 6, 0.5, setpoint=self.transform_angle_to_centerangle(
            self.transform_heading_to_angle(self.set_heading)))
        self.servo_pid.output_limits = (-90, 90)
        self.servo_pid.sample_time = 0.1
        # Max safe speed = 0.3,  Slow = 0.1,  AVG = 0.2
        self.motor_pid = PID(1, 0, 0, setpoint=0.1)
        self.motor_pid.output_limits = (0, 0.1)
        self.motor_pid.sample_time = 0.1

        # Telemetry
        self.path = "data/" + datetime.now().strftime("%d%m%Y %H:%M:%S")
        if not os.path.exists(self.path):
            os.makedirs(self.path)

        # Threads
        print("Starting threads...")
        self.stop_event = Event()
        self.threads_start()

    def threads_start(self):
        pipe_sensor_imu = deque()
        pipe_sensor_encoder = deque()
        pipe_sensor_stats = deque()
        pipe_pid_servo = deque()
        pipe_pid_motor = deque()

        thread_producer_sensor_imu = ProducerThread(
            pipe_sensor_imu, self.stop_event, self.handle_produce_sensor_imu, thread_type="TIMED_LOOP", frequency=100)
        thread_producer_sensor_encoder = ProducerThread(
            pipe_sensor_encoder, self.stop_event, self.handle_produce_sensor_encoder, thread_type="SEQUENCE")
        thread_producer_sensor_stats = ProducerThread(
            pipe_sensor_stats, self.stop_event, self.handle_produce_sensor_stats, thread_type="TIMED_LOOP", frequency=1)
        thread_producer_pid_servo = ProducerThread(
            pipe_pid_servo, self.stop_event, self.handle_produce_pid_servo, thread_type="LOOP")
        thread_producer_pid_motor = ProducerThread(
            pipe_pid_motor, self.stop_event, self.handle_produce_pid_motor, thread_type="LOOP")

        thread_reader_sensor_imu = ReaderThread(
            pipe_sensor_imu, self.stop_event, self.handle_read_sensor_imu, thread_type="TIMED_LOOP", frequency=10)
        thread_reader_sensor_encoder = ReaderThread(
            pipe_sensor_encoder, self.stop_event, self.handle_read_sensor_encoder, thread_type="TIMED_LOOP", frequency=10)

        thread_consumer_sensor_imu = ConsumerThread(
            pipe_sensor_imu, self.stop_event, self.handle_consume_sensor_imu, thread_type="TIMED_LOOP", frequency=0.1)
        thread_consumer_sensor_encoder = ConsumerThread(
            pipe_sensor_encoder, self.stop_event, self.handle_consume_sensor_encoder, thread_type="TIMED_LOOP", frequency=0.1)
        thread_consumer_sensor_stats = ConsumerThread(
            pipe_sensor_stats, self.stop_event, self.handle_consume_sensor_stats, thread_type="TIMED_LOOP", frequency=0.1)

        thread_producer_sensor_imu.start()
        thread_producer_sensor_encoder.start()
        thread_producer_sensor_stats.start()

        thread_reader_sensor_imu.start()
        thread_reader_sensor_encoder.start()

        thread_consumer_sensor_imu.start()
        thread_consumer_sensor_encoder.start()
        thread_consumer_sensor_stats.start()

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
        
    def handle_produce_sensor_encoder(self, value):
        return value

    def handle_produce_sensor_stats(self):
        with jtop() as jetson:
            return jetson.stats
        
    def handle_produce_pid_servo(self):
        return self.servo_pid(self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.heading)))
        
    def handle_produce_pid_motor(self):
        return self.motor_pid(self.velocity)
    

    def handle_read_sensor_imu(self, item):
        self.heading = item["euler"][0]
        
    def handle_read_sensor_encoder(self, item):        
        self.current_position = item * (8.1/100)
        self.velocity = (self.current_position - self.previous_position) / (1/10)
        self.previous_position = self.current_position
        print("speed: ", self.velocity)

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


mcQueen = Mcqueen()