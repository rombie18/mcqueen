import time
import logging
import signal
import traceback
import os
import board
import Jetson.GPIO as GPIO
import pyjoystick

from collections import deque
from threading import Event
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as MOTOR
from busio import I2C
from simple_pid import PID
from pyjoystick.sdl2 import run_event_loop

from customthreads import IMUThread, EncoderThread, StatsThread, ImageProcessingThread, DataCollectionThread

class McQueen:
    def __init__(self):
        # Init logger
        logging.getLogger()
        logging.basicConfig(level=logging.DEBUG, format="%(asctime)s | %(levelname)s | %(threadName)s | %(message)s")
        
        # Init variables
        self.state = "INITIALISING"
        self.control = "MANUAL"
        
        self.flag_initialised = False
        
        self.velocity = 0
        self.set_velocity = 0
        self.heading = 0
        self.set_heading = 0
        
        self._current_encoder = None
        self._previous_encoder = None
        
        # Init threads
        self.threads_init()
        
        # Init termination handlers
        signal.signal(signal.SIGINT, self.safe_stop)
        signal.signal(signal.SIGTERM, self.safe_stop)
        
        # Actuators
        logging.info("Initialising actuators...")
        bus_i2c_1 = I2C(board.SCL, board.SDA)
        pwmdriver = PCA9685(bus_i2c_1)
        pwmdriver.frequency = 50
        # Steering servo rc car, T: 20.08ms, PW: (920µs to 2120µs) -> (1320µs to 1720µs) to limit dragging against axis
        # angle 0 =~ 15° right, angle 180 =~ 15° left
        self.actuator_servo = MOTOR.Servo(
            pwmdriver.channels[0], min_pulse=1220, max_pulse=1820)
        # Motor ESC rc car, T: 20.08ms, PW: (1000µs to 2000µs)
        self.actuator_motor = MOTOR.ContinuousServo(
            pwmdriver.channels[1], min_pulse=1000, max_pulse=2000)

        # Controllers
        logging.info("Initialising PID controllers...")
        # PID: throttle 0.3 -> 3.5, 6, 0.5
        self.servo_pid = PID(3.5, 0, 0.5, setpoint=self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading)))
        self.servo_pid.output_limits = (-90, 90)
        self.servo_pid.sample_time = 0.1
        # Max safe speed = 0.3,  Slow = 0.1,  AVG = 0.2
        self.motor_pid = PID(1, 0, 0, setpoint=0.1)
        self.motor_pid.output_limits = (0, 0.2)
        self.motor_pid.sample_time = 0.1

        logging.info("Starting controller")
        mngr = pyjoystick.ThreadEventManager(event_loop=run_event_loop, add_joystick=self.controller_add, remove_joystick=self.controller_remove, handle_key_event=self.controller_process)
        mngr.start()

        logging.info("Starting all threads")
        self.threads_start()
        
        logging.info("Starting main loop")
        try:
            while not self.stop_event.is_set():
                self.process_flags()
                
                #TODO improve state machine
                if self.state == "INITIALISING":
                    # Start all thread calulations when everything is initialised
                    if self.flag_initialised:
                        logging.info("Releasing all threads.")
                        self.pause_event.clear()
                        self.state = "RUNNING"
                        
                if self.state == "RUNNING":
                    self.process_calculations()
                    if self.control == "AUTO":
                        self.process_control_loops()
                    
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logging.fatal(e)
            traceback.print_exc()
        finally:
            self.safe_stop()
            
            
    def safe_stop(self):
        logging.info("Safe stop")
        self.threads_stop()
        self.actuator_motor.throttle = 0
        self.actuator_servo.angle = self.transform_heading_to_angle(0)
        GPIO.cleanup()

    ### Control loops ###
    def process_control_loops(self):
        self.__cycle_loop_motor()
        self.__cycle_loop_steering()

    def __cycle_loop_motor(self):
        self.actuator_motor.throttle = self.motor_pid(self.velocity)

    def __cycle_loop_steering(self):
        self.actuator_servo.angle = self.transform_centerangle_to_angle(self.servo_pid(
            self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.heading))))

    ### Calculations ###
    def process_calculations(self):
        self.__calculate_velocity()
        self.__calculate_heading()
            
    def __calculate_velocity(self):
        # 162cm = 20 cycles op as encoder ==> per cycle = 8.1cm, 265 counts per cycle ==> per count = 0.3056603773584905660377358490566cm
        if len(self.pipe_encoder) > 2:
            self._current_encoder = self.pipe_encoder[-1]
            if self._previous_encoder != None and self._current_encoder["time_epoch"] != self._previous_encoder["time_epoch"]:
                self.velocity = -(self._current_encoder["position"] - self._previous_encoder["position"]) * 3.0566 / ((self._current_encoder["time_epoch"] - self._previous_encoder["time_epoch"]) * 1000)
            self._previous_encoder = self._current_encoder
        
    def __calculate_heading(self):
        if len(self.pipe_imu) != 0:
            self.heading = self.pipe_imu[-1]["euler"][0]    
            
    ### Flags ###
    def process_flags(self):
        self.__flag_initialised()

    def __flag_initialised(self):
        for init_event in self.init_events:
            if not init_event.is_set():
                self.flag_initialised = False
                return
        self.flag_initialised = True
        
    ### Helper functions ###
    def transform_heading_to_angle(self, heading):
        if heading > 180:
            if heading > 270:
                return -heading + 450
            else:
                return 180
        else:
            if heading < 90:
                return -heading + 90
            else:
                return 0

    def transform_angle_to_centerangle(self, angle):
        return angle - 90

    def transform_centerangle_to_angle(self, centerangle):
        return centerangle + 90

    ### Controller functions ###
    def controller_add(self, joy):
        logging.debug('Controller connected: ', str(vars(joy)))

    def controller_remove(self, joy):
        logging.debug('Controller disconnected: ', str(vars(joy)))
        # Robot sould stop here or at least continue in a very slow safe mode
        logging.warning('Controller disconnected, no controls available. Stopping for safety!')
        self.stop_event.set()

    def controller_process(self, key):
        try:
            logging.debug("Controller key event: " + str(vars(joy)))
            if key.keytype == "Axis" and key.number == 0:
                # Left joystick, left - right
                # Steering
                self.actuator_servo.angle = -key.raw_value * 90 + 90

            if key.keytype == "Axis" and key.number == 4:
                # Right trigger button
                # Throttle
                self.actuator_motor.throttle = key.raw_value * self.motor_pid.output_limits[1]

            if key.keytype == "Axis" and key.number == 3:
                # Right trigger button
                # Throttle
                self.actuator_motor.throttle = -key.raw_value

            if key.keytype == "Button" and key.number == 0 and key.raw_value == 1:
                # Pink square button
                # Change mode
                if self.pid_control:
                    self.pid_control = False
                    self.actuator_motor.throttle = 0
                    self.actuator_servo.angle = 90
                else:
                    self.pid_control = True
                print("PID control mode:", self.pid_control)

            if key.keytype == "Button" and key.number == 2 and key.raw_value == 1:
                # Red circle button
                # Safe stop
                self.stop = True

            if key.keytype == "Button" and key.number == 3 and key.raw_value == 1:
                # Green triangle button
                # Start
                self.start = not self.start

            if key.keytype == "Hat" and key.number == 0 and key.raw_value == 1:
                # Left hat up
                # Increase speed limit
                self.motor_pid.output_limits = (0, self.motor_pid.output_limits[1] + 0.05)

            if key.keytype == "Hat" and key.number == 0 and key.raw_value == 4:
                # Left hat down
                # Decrease speed limit
                self.motor_pid.output_limits = (0, self.motor_pid.output_limits[1] - 0.05)

            if key.keytype == "Hat" and key.number == 0 and key.raw_value == 8:
                print(vars(key))
                # Left hat left
                # Decrease PID heading
                self.set_heading = self.set_heading - 5
                self.servo_pid.setpoint = self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading))
                print("PID heading set to ", self.set_heading)

            if key.keytype == "Hat" and key.number == 0 and key.raw_value == 2:
                # Left hat right
                # Increase PID heading
                self.set_heading = self.set_heading + 5
                self.servo_pid.setpoint = self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading))
                print("PID heading set to ", self.set_heading)

        except Exception as e:
            print("-----------CONTORLLER ERROR-----------")
            print(e)
            print("-----------------CEND-----------------")

    ### Threading ###
    def threads_init(self):
        self.stop_event = Event()
        self.pause_event = Event()
        self.pause_event.set()
        
        self.init_events = []
        self.init_event_imu = Event()
        self.init_event_encoder = Event()
        self.init_event_stats = Event()
        self.init_event_imageprocessing = Event()
        self.init_event_datacollection = Event()
        self.init_events.append(self.init_event_imu)
        self.init_events.append(self.init_event_encoder)
        self.init_events.append(self.init_event_stats)
        self.init_events.append(self.init_event_imageprocessing)
        self.init_events.append(self.init_event_datacollection)

        self.pipe_imu = deque(maxlen=1000)
        self.pipe_encoder = deque(maxlen=1000)
        self.pipe_stats = deque(maxlen=1000)
        self.pipe_controller = deque(maxlen=1000)
        self.pipe_imageprocessing = deque(maxlen=1000)

        self.pipes = {
            'imu': self.pipe_imu,
            'encoder': self.pipe_encoder,
            'stats': self.pipe_stats,
            'controller': self.pipe_controller,
            'imageprocessing': self.pipe_imageprocessing,
        }

        self.threads = []
        self.threads.append(IMUThread(self.pipe_imu, self.stop_event, self.init_event_imu, self.pause_event))
        self.threads.append(EncoderThread(self.pipe_encoder, self.stop_event, self.init_event_encoder, self.pause_event))
        self.threads.append(StatsThread(self.pipe_stats, self.stop_event, self.init_event_stats, self.pause_event))
        self.threads.append(ImageProcessingThread(self.pipe_imageprocessing, self.stop_event, self.init_event_imageprocessing, self.pause_event))
        self.threads.append(DataCollectionThread(None, self.stop_event, self.init_event_datacollection, self.pause_event, self.pipes))
        
    def threads_start(self):
        for thread in self.threads:
            thread.start()

    def threads_stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join()
            
            
mcqueen = McQueen()