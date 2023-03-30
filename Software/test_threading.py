from collections import deque
import time
from threading import Thread, Event
import logging
import random

class ProducerThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_produce, argument):
        super(ProducerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_produce = handle_produce
        self.argument = argument

    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            value = self.handle_produce(self.argument)
            self.pipe.appendleft(value)
            logging.debug("Produced: %s -> %s", str(value), str(self.pipe))
            time.sleep(self.period - ((time.time() - starttime) % self.period))


class ReaderThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_read, argument):
        super(ReaderThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_read = handle_read
        self.argument = argument

    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            if len(self.pipe) > 0:
                self.handle_read(self.argument, self.pipe[0])
                logging.debug("Read: %s -> %s",
                              str(self.pipe[0]), str(self.pipe))
            else:
                logging.debug("Read: Empty pipe")
            time.sleep(self.period - ((time.time() - starttime) % self.period))


class ConsumerThread(Thread):
    def __init__(self, pipe, stop_event, frequency, handle_consume, argument):
        super(ConsumerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.period = 1 / frequency
        self.handle_consume = handle_consume
        self.argument = argument

    def run(self):
        starttime = time.time()
        while not self.stop_event.is_set():
            if len(self.pipe) > 0:
                values = []
                while len(self.pipe) > 0:
                    values.append(self.pipe.pop())
                self.handle_consume(self.argument, values)
                logging.debug("Consumed: %s -> %s",
                              str(values), str(self.pipe))
                time.sleep(self.period -
                           ((time.time() - starttime) % self.period))
        values = []
        while len(self.pipe) > 0:
            values.append(self.pipe.pop())
        self.handle_consume(self.argument, values)
        logging.debug("Consumed: %s -> %s", str(values), str(self.pipe))
        
        
logging.basicConfig(level=logging.INFO,  format="%(asctime)s - %(message)s")

pipe= deque()
stop_event = Event()

def handle_produce(arg):
    value = random.randint(0, 9)
    logging.info("Produced value: %d", value)
    return value

def handle_read(arg, value):
    logging.info("Read value: %d", value)

def handle_consume(arg, values):
    logging.info("Consumed values: %s", str(values))

thread_producer = ProducerThread(
    pipe, stop_event, 2, handle_produce, None)

thread_reader = ReaderThread(
    pipe, stop_event, 3, handle_read, None)

thread_consumer = ConsumerThread(
    pipe, stop_event, 0.1, handle_consume, None)

thread_producer.start()
thread_reader.start()
thread_consumer.start()