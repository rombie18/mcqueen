import time
import logging
from collections import deque
from threading import Event
from customthreads import IMUThread, EncoderThread, StatsThread

logging.getLogger()
logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(threadName)-16s | %(message)s")
stop_event = Event()

pipe_sensor_imu = deque(maxlen=100)
pipe_sensor_encoder = deque(maxlen=100)
pipe_sensor_stats = deque(maxlen=100)

threads = []
threads.append(IMUThread(pipe_sensor_imu, stop_event))
threads.append(EncoderThread(pipe_sensor_encoder, stop_event))
threads.append(StatsThread(pipe_sensor_stats, stop_event))

def threads_start():
    for thread in threads:
        thread.start()

def threads_stop():
    stop_event.set()
    for thread in threads:
        thread.join()

threads_start()
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    logging.debug("KeyboardInterrupt: Stopping program")
except Exception as e:
    logging.fatal(e)
finally:
    logging.info("Signaling threads to stop")

    
