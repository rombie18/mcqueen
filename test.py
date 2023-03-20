import logging
import threading
import random
from threading import Event, Thread
import time
from collections import deque

logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(message)s")


def produce(stop, deque, frequency):
    starttime = time.time()
    while not stop.is_set():
        value = random.randint(0, 9)
        deque.appendleft(value)
        logging.debug("Produced: %d -> %s", value, str(deque))
        time.sleep((1/frequency) - ((time.time() - starttime) % (1/frequency)))


def read(stop, deque, frequency):
    starttime = time.time()
    while not stop.is_set():
        value = deque[0]
        logging.debug("Read: %d -> %s", value, str(deque))
        time.sleep((1/frequency) - ((time.time() - starttime) % (1/frequency)))


def consume(stop, deque, frequency, min_items):      
    starttime = time.time()
    while not stop.is_set():
        if len(deque) > min_items:
            values = []
            while len(deque) > min_items:
                values.append(deque.pop())
            # Call store function here
            logging.debug("Consumed: %s -> %s", values, str(deque))

        time.sleep((1/frequency) - ((time.time() - starttime) % (1/frequency)))

    values = []
    while len(deque) > 0:
        values.append(deque.pop())
    logging.debug("Consumed: %s -> %s", values, str(deque))
    # Call store function here


logging.info("Starting Threads...\n")
logging.info("Press Ctrl+C to interrupt the execution\n")

shared_queue = deque()
event = Event()

Thread(target=produce, kwargs={'stop': event, 'deque': shared_queue, 'frequency': 5}).start()
time.sleep(1)
Thread(target=read, kwargs={'stop': event, 'deque': shared_queue, 'frequency': 10}).start()
Thread(target=read, kwargs={'stop': event, 'deque': shared_queue, 'frequency': 10}).start()
time.sleep(5)
Thread(target=consume, kwargs={'stop': event, 'deque': shared_queue, 'frequency': 1, 'min_items': 10}).start()

time.sleep(5)
event.set()