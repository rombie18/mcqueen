import time
import logging
from collections import deque
from threading import Event
from customthreads import IMUThread, EncoderThread

logging.getLogger()
logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(threadName)-16s | %(message)s")
stop_event = Event()

pipe_sensor_imu = deque(maxlen=100)
thread_producer_sensor_imu = IMUThread(pipe_sensor_imu, stop_event)
thread_producer_sensor_imu.start()

pipe_sensor_encoder = deque(maxlen=100)
thread_producer_sensor_encoder = EncoderThread(pipe_sensor_encoder, stop_event)
thread_producer_sensor_encoder.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    logging.debug("KeyboardInterrupt: Stopping program")
except Exception as e:
    logging.fatal(e)
finally:
    logging.info("Signaling threads to stop")
    stop_event.set()
    thread_producer_sensor_imu.join()
    thread_producer_sensor_encoder.join()