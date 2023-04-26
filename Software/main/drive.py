import sys
sys.path.append("/home/mcqueen/mcqueen/Software/libs/encoder")
sys.path.append("/home/mcqueen/mcqueen/Software/libs/tis")

import board
import time
import json
import cv2
import numpy as np

from simple_pid import PID
from busio import I2C
from adafruit_bno055 import BNO055_I2C
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as MOTOR
import pyjoystick
from pyjoystick.sdl2 import run_event_loop

from encoder import Encoder
from tis import TIS, SinkFormats


class McQueen:
    # Main methods
    def __init__(self):
        # Variables
        self.velocity = 0
        self.heading = 0
        self.stop = False
        self.start = False
        self.pid_control = False
        self.set_heading = 0
        
        self.current_position = 0
        self.previous_position = 0
        self.current_time = 0
        self.previous_time = 0

        self.X_lb=0
        self.Y_lb=0
        self.X_rb=0
        self.Y_rb=0
        self.lower = np.array([150, 100, 20])
        self.upper = np.array([180, 255, 255])
        self.h1 = 0
        self.h2 = 0

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
        self.sensor_encoder = Encoder(board.D6, board.D12)
        # 162cm = 20 cycles op as encoder ==> per cycle = 8.1cm, 265 counts per cycle 

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
        # PID: throttle 0.3 -> 3.5, 6, 0.5
        self.servo_pid = PID(3.5, 0, 0.5, setpoint=self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading)))
        self.servo_pid.output_limits = (-90, 90)
        self.servo_pid.sample_time = 0.1
        # Max safe speed = 0.3,  Slow = 0.1,  AVG = 0.2
        self.motor_pid = PID(1, 0, 0, setpoint=0.1)
        self.motor_pid.output_limits = (0, 0.2)
        self.motor_pid.sample_time = 0.1

        # Image processing
        print("Initialising image processing...")
        self.Tis = TIS()
        self.Tis.open_device("02320237", 1280, 720, "60/1", SinkFormats.BGRA, False)

        with open('camera_properties.json', 'r') as file:
            data = json.load(file)
            for key, item in data.items():
                self.Tis.set_property(key, item)

        try:
            self.Tis.set_property("TriggerMode","Off")
        except Exception as error:
            print(error)

        self.Tis.start_pipeline()


        try:
            print("Starting main loop...")
            self.main_loop()
        except Exception as e:
            print("-----------ERROR-----------")
            print(e)
            print("------------END------------")
        finally:
            print("Handling safe stop...")
            self.safe_stop()

    def main_loop(self):

        mngr = pyjoystick.ThreadEventManager(event_loop=run_event_loop, add_joystick=self.controller_add, remove_joystick=self.controller_remove, handle_key_event=self.controller_process)
        mngr.start()

        while not self.stop:
            time_start = time.time()
            self.calculate_velocity()
            self.calculate_heading()
            if self.pid_control:
                self.cycle_loop_motor()
                self.cycle_loop_steering()

            if self.start:
                self.actuator_servo.angle = self.do_image_proccess()

            time_end = time.time()
            print(time_end - time_start)

    def safe_stop(self):
        self.actuator_motor.throttle = 0
        self.actuator_servo.angle = self.transform_heading_to_angle(0)
        self.Tis.stop_pipeline()
        cv2.destroyAllWindows()

    # Control loops

    def cycle_loop_motor(self):
        self.actuator_motor.throttle = self.motor_pid(self.velocity)

    def cycle_loop_steering(self):
        self.actuator_servo.angle = self.transform_centerangle_to_angle(self.servo_pid(
            self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.heading))))

    def calculate_velocity(self):
        self.velocity = 0.1

    def calculate_heading(self):
        self.heading = self.sensor_imu.euler[0]

    # Helper functions
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

    # Controller functions
    def controller_add(self, joy):
        print('Controller connected:', joy)

    def controller_remove(self, joy):
        print('Controller disconnected:', joy)
        # Robot sould stop here or at least continue in a very slow safe mode
        self.stop = True

    def controller_process(self, key):
        try:
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

    def do_image_proccess(self):
        angle = 90
        if self.Tis.snap_image(1):
            video = self.Tis.get_image()

            hight, width, _ = video.shape #Get resolution

            video_hsv = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) #Converting BGR image to HSV format

            mask_video = cv2.inRange(video_hsv, self.lower, self.upper) # Masking the image to find our color
            
            mask_leftBand = mask_video[0:hight, int(250):int(300)] #Crop right band
            mask_rightBand = mask_video[0:hight, int(width-300):int(width-250)] #crop left band

            mask_contours_leftBand, hierarchy = cv2.findContours(mask_leftBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (leftband)
            mask_contours_rightBand, hierarchy = cv2.findContours(mask_rightBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (rightband)

            # Finding position of all contours of the bands
            if len(mask_contours_leftBand or mask_contours_rightBand) != 0:
                for i_lb, k_rb in zip(mask_contours_leftBand, mask_contours_rightBand):
                    if cv2.contourArea(i_lb) > 500:
                        x1, y1, w1, self.h1 = cv2.boundingRect(i_lb)
                        X_lb = int(x1+(w1/2)) #calculate center of rectangle
                        Y_lb = int(y1+(self.h1/2))
                        cv2.rectangle(video, (x1+250, y1), (x1 + w1+250, y1 + self.h1), (0, 0, 255), 3) #drawing rectangle
                    
                    if cv2.contourArea(k_rb) > 500:
                        x2, y2, w2, self.h2 = cv2.boundingRect(k_rb)
                        X_rb = int(width-(x2+(w2/2))) #calculate center of rectangle
                        Y_rb = int(y2+(self.h2/2))
                        cv2.rectangle(video, ((width-x2-300), y2), (((width-x2-300) + w2), (y2 + self.h2)), (255, 0, 0), 3) #drawing rectangle (formula needed, otherwise it will add it on the left side)

                    
                diff = 1
                if self.h2 != 0:
                    diff = self.h1/self.h2
                if (diff > 0.9) and (diff < 1.1):
                    print("Go straight")
                if diff > 1.1: #h1>h2
                    norm = (self.h1-self.h2)/self.h1
                    angle = 90*norm
                    print("Go right, with angle:", angle)
                if diff < 0.9: #h1<h2
                    norm = 1-((self.h2-self.h1)/self.h2)
                    angle = 90+90*norm
                    print("Go left, with angle:", angle)

        return angle

mcqueen = McQueen()
