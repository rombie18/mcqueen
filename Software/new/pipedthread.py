import time
import logging
from threading import Thread

class ProducerThread(Thread):
    def __init__(self, pipe, stop_event, handle_produce, thread_type="TIMED_LOOP", frequency=1):
        super(ProducerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.handle_produce = handle_produce
        self.thread_type = thread_type
        self.period = 1 / frequency

    def run(self):
        if self.thread_type == "SEQUENCE":
            value = self.handle_produce()
            self.pipe.appendleft(value)
            logging.debug("Produced: %s -> %s", str(value), str(self.pipe))

        if self.thread_type == "LOOP":
            while not self.stop_event.is_set():
                value = self.handle_produce()
                self.pipe.appendleft(value)
                logging.debug("Produced: %s -> %s", str(value), str(self.pipe))

        if self.thread_type == "TIMED_LOOP":
            starttime = time.time()
            while not self.stop_event.is_set():
                value = self.handle_produce()
                self.pipe.appendleft(value)
                logging.debug("Produced: %s -> %s", str(value), str(self.pipe))
                time.sleep(self.period -
                           ((time.time() - starttime) % self.period))


class ReaderThread(Thread):
    def __init__(self, pipe, stop_event, handle_read, thread_type="TIMED_LOOP", frequency=1):
        super(ReaderThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.handle_read = handle_read
        self.thread_type = thread_type
        self.period = 1 / frequency

    def run(self):
        if self.thread_type == "SEQUENCE":
            if len(self.pipe) > 0:
                self.handle_read(self.pipe[0])
                logging.debug("Read: %s -> %s",
                              str(self.pipe[0]), str(self.pipe))
            else:
                logging.debug("Read: Empty pipe")

        if self.thread_type == "LOOP":
            while not self.stop_event.is_set():
                if len(self.pipe) > 0:
                    self.handle_read(self.pipe[0])
                    logging.debug("Read: %s -> %s",
                                  str(self.pipe[0]), str(self.pipe))
                else:
                    logging.debug("Read: Empty pipe")

        if self.thread_type == "TIMED_LOOP":
            starttime = time.time()
            while not self.stop_event.is_set():
                if len(self.pipe) > 0:
                    self.handle_read(self.pipe[0])
                    logging.debug("Read: %s -> %s",
                                  str(self.pipe[0]), str(self.pipe))
                else:
                    logging.debug("Read: Empty pipe")
                time.sleep(self.period -
                           ((time.time() - starttime) % self.period))


class ConsumerThread(Thread):
    def __init__(self, pipe, stop_event, handle_consume, thread_type="TIMED_LOOP", frequency=1):
        super(ConsumerThread, self).__init__()
        self.pipe = pipe
        self.stop_event = stop_event
        self.handle_consume = handle_consume
        self.thread_type = thread_type
        self.period = 1 / frequency
        self.pipe_buffer = 10

    def run(self):
        if self.thread_type == "SEQUENCE":
            if len(self.pipe) > self.pipe_buffer:
                values = []
                while len(self.pipe) > self.pipe_buffer:
                    values.append(self.pipe.pop())
                self.handle_consume(values)
                logging.debug("Consumed: %s -> %s",
                              str(values), str(self.pipe))
            values = []
            while len(self.pipe) > 0:
                values.append(self.pipe.pop())
            self.handle_consume(values)
            logging.debug("Consumed: %s -> %s", str(values), str(self.pipe))

        if self.thread_type == "LOOP":
            while not self.stop_event.is_set():
                if len(self.pipe) > self.pipe_buffer:
                    values = []
                    while len(self.pipe) > self.pipe_buffer:
                        values.append(self.pipe.pop())
                    self.handle_consume(values)
                    logging.debug("Consumed: %s -> %s",
                                  str(values), str(self.pipe))
            values = []
            while len(self.pipe) > 0:
                values.append(self.pipe.pop())
            self.handle_consume(values)
            logging.debug("Consumed: %s -> %s", str(values), str(self.pipe))

        if self.thread_type == "TIMED_LOOP":
            starttime = time.time()
            while not self.stop_event.is_set():
                if len(self.pipe) > self.pipe_buffer:
                    values = []
                    while len(self.pipe) > self.pipe_buffer:
                        values.append(self.pipe.pop())
                    self.handle_consume(values)
                    logging.debug("Consumed: %s -> %s",
                                  str(values), str(self.pipe))
                    time.sleep(self.period -
                               ((time.time() - starttime) % self.period))
            values = []
            while len(self.pipe) > 0:
                values.append(self.pipe.pop())
            self.handle_consume(values)
            logging.debug("Consumed: %s -> %s", str(values), str(self.pipe))