import board
import time
import math

from simple_pid import PID
from busio import I2C
# from rotaryio import IncrementalEncoder
from adafruit_bno055 import BNO055_I2C
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as MOTOR
from pyjoystick.sdl2 import Key, Joystick, run_event_loop


class McQueen:
    # Main methods
    def __init__(self):
        # Variables
        self.velocity = 0
        self.heading = 0
        self.stop = False

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
        self.servo_pid = PID(3.5, 6, 0.5, setpoint=self.transform_angle_to_centerangle(self.transform_heading_to_angle(45)))
        self.servo_pid.output_limits = (-90, 90)
        self.servo_pid.sample_time = 0.1
        # Max safe speed = 0.3,  Slow = 0.1,  AVG = 0.2
        self.motor_pid = PID(1, 0, 0, setpoint=0.15)
        self.motor_pid.output_limits = (0, 0.15)
        self.motor_pid.sample_time = 0.1

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
        run_event_loop(self.controller_add, self.controller_remove, self.controller_process)
        print("ooook")
        while not self.stop:
            self.calculate_velocity()
            self.calculate_heading()
            self.cycle_loop_motor()
            self.cycle_loop_steering()

    def safe_stop(self):
        self.actuator_motor.throttle = 0
        self.actuator_servo.angle = self.transform_heading_to_angle(0)

    # Control loops

    def cycle_loop_motor(self):
        #self.actuator_motor.throttle = self.motor_pid(self.velocity)
        self.actuator_motor.throttle = 0.3

    def cycle_loop_steering(self):
        self.actuator_servo.angle = self.transform_centerangle_to_angle(self.servo_pid(
            self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.heading))))

    def calculate_velocity(self):
        if self.motor_pid.setpoint < self.velocity:
            self.velocity = self.velocity + 0.00001

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
        print('Controller connected', joy)

    def controller_remove(self, joy):
        print('Controller disconnected', joy)
        # Robot sould stop here or at least continue in a very slow safe mode

    def controller_process(self, key):
        if key.keytype == "Axis" and key.number == 2:
            # Right joystick, left - right
            # Steering
            self.actuator_servo.angle = key.raw_value * 90 + 90

        if key.keytype == "Axis" and key.number == 4:
            # Right trigger button
            # Throttle
            self.actuator_motor.thottle = key.raw_value

        if key.keytype == "Button" and key.number == 2:
            # Pink square button
            # Change mode
            print("Change to PID control")
            print(vars(key))

        if key.keytype == "Button" and key.number == 2:
            # Red circle button
            # Safe stop
            self.stop = True

        if key.keytype == "Button" and key.number == 3:
            # Green triangle button
            # Start
            print("Start")
            print(vars(key))

        if key.keytype == "Hat" and key.number == 0 and key.raw_value == 1:
            # Left hat up
            # Increase speed limit
            self.motor_pid.output_limits = (0, self.motor_pid.output_limits[1] + 0.05)

        if key.keytype == "Hat" and key.number == 0 and key.raw_value == 4:
            # Left hat down
            # Decrease speed limit
            self.motor_pid.output_limits = (0, self.motor_pid.output_limits[1] - 0.05)


mcqueen = McQueen()