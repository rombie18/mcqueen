import time
import pprint
from collections import deque
from threading import Event
from IMUThread import IMUThread

pp = pprint.PrettyPrinter(indent=4)

stop_event = Event()

pipe_sensor_imu = deque(maxlen=100)
thread_producer_sensor_imu = IMUThread(pipe_sensor_imu, stop_event)
thread_producer_sensor_imu.start()

try:
    while True:
        if len(pipe_sensor_imu) > 0:
            pp.pprint(pipe_sensor_imu[-1])
        time.sleep(1)
except Exception as e:
            print("-----------ERROR-----------")
            print(e)
            print("------------END------------")
finally:
    print("Stopping all threads...")
    stop_event.set()