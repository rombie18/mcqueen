from enum import Enum, auto

class States(Enum):
    INITIALISING = auto()
    IDLE = auto()
    STARTING = auto()
    NOMINAL = auto()
    DEGRADED = auto()
    CORRECTING = auto()
    RECOVERING = auto()
    STOPPING = auto()
    FAULT = auto()
    HALTING = auto()
    
class McQueen():
    def __init__(self):
        self.state = States.INITIALISING
        
    def next_state_logic(self):
        if self.state == States.INITIALISING:
            if self.flag_fault:
                self.state = States.FAULT
            elif self.flag_initialised:
                self.state = States.IDLE
                
        if self.state == States.IDLE:
            if self.flag_halt:
                self.state = States.HALTING
            elif self.wait_for_start_button:
                self.state = States.STARTING
                
        if self.state == States.STARTING:
            if self.flag_stop:
                self.state = States.STOPPING
            elif self.image_proc_active and self.robot_up_to_speed:
                self.state = States.NOMINAL
                
        if self.state == States.NOMINAL:
            if self.flag_stop:
                self.state = States.STOPPING
            elif self.lost_controller_conn:
                self.state = States.DEGRADED
            elif self.cond_correcting:
                self.state = States.CORRECTING
                
        if self.state == States.DEGRADED:
            if self.flag_stop:
                self.state = States.STOPPING
            elif self.controller_reconnected:
                self.state = States.NOMINAL
                
        if self.state == States.CORRECTING:
            if self.flag_stop:
                self.state = States.STOPPING
            elif self.cond_recovering:
                self.state = States.RECOVERING
            elif self.cond_nominal:
                self.state = States.NOMINAL
                
        if self.state == States.RECOVERING:
            if self.flag_stop:
                self.state = States.STOPPING
            elif self.flag_fault:
                self.state = States.FAULT
            elif self.cond_correcting:
                self.state = States.CORRECTING
                
        if self.state == States.STOPPING:
            if self.robot_stopped and self.everything_saved:
                self.state = States.IDLE
                
        if self.state == States.FAULT:
            if self.error_logged and self.deinitialised and self.allowed_restart:
                self.state = States.INITIALISING
            
        if self.state == States.HALTING:
            self.flag_halt

    def output_logic(self):
        if self.state == States.INITIALISING:
            # Init sensors
            pass
                
        if self.state == States.IDLE:
            # Do nothing
            pass
                
        if self.state == States.STARTING:
            # Throttle up, start improc, start calculations
            pass
                
        if self.state == States.NOMINAL:
            # Run normal functions
            pass
                
        if self.state == States.DEGRADED:
            # Limit speed or stop
            pass
                
        if self.state == States.CORRECTING:
            # Run other improc algoritm
            pass
                
        if self.state == States.RECOVERING:
            # Do recovering manuever
            pass
                
        if self.state == States.STOPPING:
            # Throttle down motors, stop image proc
            pass
                
        if self.state == States.FAULT:
            # Report error and deinit
            pass
            
        if self.state == States.HALTING:
            # Turn off Jetson Nano
            pass