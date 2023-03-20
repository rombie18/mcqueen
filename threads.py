import time
from threading import Event, Thread
from collections import deque
#from jtop import jtop
import os
import csv
import board
import time
import math

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
        self.velocity = 0
        self.heading = 0
        self.stop = False
        self.pid_control = False
        self.set_heading = 0

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
        # self.sensor_encoder = IncrementalEncoder(board.D10, board.D9)

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
        self.servo_pid = PID(3.5, 6, 0.5, setpoint=self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading)))
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
        self.stop_event = Event()
        self.startThreads()

    def startThreads(self):

        def handle_produce_sensor_imu():
            return self.sensor_imu

        def handle_read_sensor_imu(value):
            print(value)
            
        def handle_consume_sensor_imu(values):
            filename = "imu.csv"
            with open(self.path + "/" + filename, 'w') as file:
                writer = csv.writer(file)
                writer.writerow(["to", "do"])
                for value in values:
                    writer.writerow(["to", "do"])
                    
        pipe_sensor_imu = deque()
        thread_producer_sensor_imu = ProducerThread(pipe_sensor_imu, self.stop_event, 100, handle_produce=handle_produce_sensor_imu)
        thread_reader_sensor_imu = ReaderThread(pipe_sensor_imu, self.stop_event, 100, handle_read=handle_read_sensor_imu)
        thread_consumer_sensor_imu = ConsumerThread(pipe_sensor_imu, self.stop_event, 0.5, handle_consume=handle_consume_sensor_imu)

        thread_producer_sensor_imu.start()
        thread_reader_sensor_imu.start()
        thread_consumer_sensor_imu.start()


class ProducerThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_produce):
        super(ProducerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_produce = handle_produce
 
    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            value = self.handle_produce()
            self.pipe.appendleft(value)
            time.sleep(self.period - ((time.time() - starttime) % self.period))
 
class ReaderThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_read):
        super(ReaderThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_read = handle_read
 
    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set() and len(self.pipe) > 0:
            self.handle_read(self.pipe[0])
            time.sleep(self.period - ((time.time() - starttime) % self.period))

class ConsumerThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_consume):
        super(ConsumerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_consume = handle_consume
 
    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            if len(self.pipe) > 10:
                values = []
                while len(self.pipe) > 10:
                    values.append(self.pipe.pop())
                self.handle_consume(values)
            time.sleep(self.period - ((time.time() - starttime) % self.period))
        values = []
        while len(self.pipe) > 0:
            values.append(self.pipe.pop())
        self.handle_consume(values)

mcQueen = Mcqueen()