from pyjoystick.sdl2 import Key, Joystick, run_event_loop

def print_add(joy):
    print('Controller connected', joy)

def print_remove(joy):
    print('Controller disconnected', joy)
    # Robot sould stop here or at least continue in a very slow safe mode

def key_received(key):

    if key.keytype == "Axis" and key.number == 2:
        # Right joystick, left - right
        # Steering
        print("Steering")
        print(vars(key))

    if key.keytype == "Axis" and key.number == 4:
        # Right trigger button
        # Throttle
        print("Throttle")
        print(vars(key))

    if key.keytype == "Button" and key.number == 2:
        # Red circle button
        # Safe stop
        print("Safe stop")
        print(vars(key))

    if key.keytype == "Button" and key.number == 3:
        # Green triangle button
        # Start
        print("Start")
        print(vars(key))

    if key.keytype == "Hat" and key.number == 0 and key.raw_value == 1:
        # Left hat up
        # Increase speed limit
        print("Increase speed limit")
        print(vars(key))

    if key.keytype == "Hat" and key.number == 0 and key.raw_value == 4:
        # Left hat down
        # Decrease speed limit
        print("Decrease speed limit")
        print(vars(key))

run_event_loop(print_add, print_remove, key_received)