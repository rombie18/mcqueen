import time
from collections import deque
from threading import Event
from IMUThread import IMUThread

stop_event = Event()

pipe_sensor_imu = deque([], maxlen=100)
thread_producer_sensor_imu = IMUThread(pipe_sensor_imu, stop_event)
thread_producer_sensor_imu.start()

try:
    while True:
        time.sleep(0.1)
except:
    print("Stopping all threads...")
finally:
    stop_event.set()