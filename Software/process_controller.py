from pyjoystick.sdl2 import Key, Joystick, run_event_loop

def print_add(joy):
    print('Controller connected', joy)

def print_remove(joy):
    print('Controller disconnected', joy)
    # Robot sould stop here or at least continue in a very slow safe mode

def key_received(key):
    print(vars(key))

run_event_loop(print_add, print_remove, key_received)