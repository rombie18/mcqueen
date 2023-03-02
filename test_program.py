import board
import time
import inquirer

from busio import I2C
# from rotaryio import IncrementalEncoder
from adafruit_bno055 import BNO055_I2C
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as MOTOR

tests = [
  inquirer.List('tests',
                message="Welke test wil je uitvoeren?",
                choices=['Servo', 'Motor', 'Gyro/IMU', 'Encoder', 'Joystick/Controller', 'Camera'],
            ),
]


class McQueen:
    # Main methods
    def __init__(self):

        response = inquirer.prompt(tests)

        if response["tests"] == "Servo":

            print("Initialising busses...")
            bus_i2c_1 = I2C(board.SCL, board.SDA)
            print("Initialising PWM driver...")
            pwmdriver = PCA9685(bus_i2c_1)
            pwmdriver.frequency = 50
            print("Initialising actuators...")
            # Steering servo rc car, T: 20.08ms, PW: (920µs to 2120µs) -> (1320µs to 1720µs) to limit dragging against axis
            # angle 0 =~ 15° right, angle 180 =~ 15° left
            self.actuator_servo = MOTOR.Servo(
                pwmdriver.channels[0], min_pulse=1220, max_pulse=1820)
            self.test_servo()

        elif response["tests"] == "Motor":

            print("Initialising busses...")
            bus_i2c_1 = I2C(board.SCL, board.SDA)
            print("Initialising PWM driver...")
            pwmdriver = PCA9685(bus_i2c_1)
            pwmdriver.frequency = 50
            print("Initialising actuators...")
            # Motor ESC rc car, T: 20.08ms, PW: (1000µs to 2000µs)
            self.actuator_motor = MOTOR.ContinuousServo(
                pwmdriver.channels[1], min_pulse=1000, max_pulse=2000)
            self.test_motor()
        
        elif response["tests"] == "Gyro/IMU":

            bus_i2c_2 = I2C(board.SCL_1, board.SDA_1)
            print("Initialising sensors...")
            self.sensor_imu = BNO055_I2C(bus_i2c_2)
            self.test_imu()

        else:
            print("Nog niet geïmplementeerd!")
        

    # Test functions
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
        self.actuator_servo.angle = 0
        time.sleep(5)
        print("Full right")
        self.actuator_servo.angle = 180
        time.sleep(5)
        print("Center")
        self.actuator_servo.angle = 90
        time.sleep(5)
        print()

    def test_motor(self):
        print()
        print("## MOTOR ##")
        print("Forwards")
        self.actuator_motor.throttle = 0.1
        time.sleep(2)
        print("Stop")
        self.actuator_motor.throttle = 0.05
        time.sleep(2)
        print("Backwards")
        self.actuator_motor.throttle = -0.1
        time.sleep(2)
        print("Brake")
        self.actuator_motor.throttle = 0.1
        time.sleep(2)
        print("Stop")
        self.actuator_motor.throttle = 0
        time.sleep(3)
        print()


mcqueen = McQueen()
