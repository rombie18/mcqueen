import time
import logging
from collections import deque
from threading import Event
from customthreads import IMUThread, EncoderThread

logging.getLogger()
logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(threadName)s: %(message)s")
stop_event = Event()

pipe_sensor_imu = deque(maxlen=100)
thread_producer_sensor_imu = IMUThread(pipe_sensor_imu, stop_event)
thread_producer_sensor_imu.start()

pipe_sensor_encoder = deque(maxlen=100)
thread_producer_sensor_encoder = EncoderThread(pipe_sensor_encoder, stop_event)
thread_producer_sensor_encoder.start()

try:
    while True:
        if len(pipe_sensor_imu) > 0:
            print(pipe_sensor_imu[-1], end="\n\n")
        if len(pipe_sensor_encoder) > 0:
            print(pipe_sensor_encoder[-1], end="\n\n")
        time.sleep(1)
except Exception as e:
            print("-----------ERROR-----------")
            print(e)
            print("------------END------------")
finally:
    print("Stopping all threads...")
    stop_event.set()