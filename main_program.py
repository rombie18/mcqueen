import board
import time
import math

from simple_pid import PID
from busio import I2C
#from rotaryio import IncrementalEncoder
from adafruit_bno055 import BNO055_I2C
from adafruit-circuitpython-pca9685 import PCA9685
from adafruit-circuitpython-motor import servo as MOTOR

class McQueen:
    ## Main methods
    def __init__(self):
        # Variables
        self.velocity = 0
        self.heading = 0

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
        #self.sensor_encoder = IncrementalEncoder(board.D10, board.D9)

        # Actuators
        print("Initialising actuators...")
        # Steering servo rc car, T: 20.08ms, PW: (920µs to 2120µs) -> (1320µs to 1720µs) to limit dragging against axis
        # angle 0 =~ 15° right, angle 180 =~ 15° left
        self.actuator_servo = MOTOR.Servo(pwmdriver.channels[0], min_pulse=1320, max_pulse=1720)
        # Motor ESC rc car, T: 20.08ms, PW: (1000µs to 2000µs)
        self.actuator_motor = MOTOR.ContinuousServo(pwmdriver.channels[1], min_pulse=1000, max_pulse=2000)

        # Controllers
        print("Initialising controllers...")
        self.servo_pid = PID(1, 0.1, 0.05, setpoint=90)
        self.servo_pid.output_limits = (0, 180)
        self.servo_pid.sample_time = 0.01
        self.motor_pid = PID(1, 0.1, 0.05, setpoint=0.5)
        self.motor_pid.output_limits = (0, 0.15)
        self.motor_pid.sample_time = 0.01

        try:
            self.test_servo()
            #print("Starting main loop...")
            #self.main_loop()
        except:
            self.safe_stop()

    def main_loop(self):
        while True:
            print("Velocity: {}, Heading: {}".format(self.velocity, self.heading))
            self.calculate_velocity()
            self.calculate_heading()
            self.cycle_loop_motor()
            self.cycle_loop_steering()
            time.sleep(0.1)

    def safe_stop(self):
        print("Handling safe stop...")
        self.actuator_motor.throttle = 0
        self.actuator_servo.angle = 90


    ## Control loops
    def cycle_loop_motor(self):
        self.actuator_motor.throttle = self.motor_pid(self.velocity)

    def cycle_loop_steering(self):
        self.actuator_servo.angle = self.servo_pid(self.transform_heading_to_angle(self.heading))

    def calculate_velocity(self):
        # TODO
        if self.velocity < 1:
            self.velocity = self.velocity + 0.005

    def calculate_heading(self):
        self.heading = self.sensor_imu.euler[0]

    ## Helper functions
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

    ## Test functions
    def test_imu(self):
        print()
        print("## IMU ##")
        print("Temperature: {} degrees C".format(self.sensor_imu.temperature))
        print("Accelerometer (m/s^2): {}".format(self.sensor_imu.acceleration))
        print("Magnetometer (microteslas): {}".format(self.sensor_imu.magnetic))
        print("Gyroscope (rad/sec): {}".format(self.sensor_imu.gyro))
        print("Euler angle: {}".format(self.sensor_imu.euler))
        print("Quaternion: {}".format(self.sensor_imu.quaternion))
        print("Linear acceleration (m/s^2): {}".format(self.sensor_imu.linear_acceleration))
        print("Gravity (m/s^2): {}".format(self.sensor_imu.gravity))
        print()

    def test_encoder(self):
        print()
        print("## ENCODER ##")
        print("Position: {}".format(self.sensor_encoder.position))
        print()

    def test_servo(self):
        print()
        print("## SERVO ##")
        print("Full left")
        self.actuator_motor.throttle = 0.2
        time.sleep(1)
        print("Full right")
        self.actuator_motor.throttle = -0.2
        time.sleep(1)
        print("Center")
        self.actuator_motor.throttle = 0
        print()

    def test_motor(self):
        print()
        print("## MOTOR ##")
        print("Forwards")
        self.actuator_servo.angle = 0
        time.sleep(1)
        print("Backwards")
        self.actuator_servo.angle = 180
        time.sleep(1)
        print("Stop")
        self.actuator_servo.angle = 90
        print()

mcqueen = McQueen()