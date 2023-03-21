import pyjoystick
from pyjoystick.sdl2 import Key, Joystick, run_event_loop

class ControllerTest:
    def __init__(self):
        mngr = pyjoystick.ThreadEventManager(event_loop=run_event_loop, add_joystick=self.controller_add, remove_joystick=self.controller_remove, handle_key_event=self.controller_process)
        mngr.start()
    
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
                print("Start")

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
                self.set_heading = self.set_heading + 5
                self.servo_pid.setpoint = self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading))

            if key.keytype == "Hat" and key.number == 0 and key.raw_value == 2:
                print(vars(key))
                # Left hat right
                # Increase PID heading
                self.set_heading = self.set_heading - 5
                self.servo_pid.setpoint = self.transform_angle_to_centerangle(self.transform_heading_to_angle(self.set_heading))
                
        except Exception as e:
            print("-----------CONTORLLER ERROR-----------")
            print(e)
            print("-----------------CEND-----------------")