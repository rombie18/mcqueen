import time
import logging
from collections import deque
from threading import Event
from customthreads import IMUThread, EncoderThread, StatsThread, ControllerThread, ImageProcessingThread

class McQueen:
    def __init__(self):
        
        logging.getLogger()
        logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(threadName)-16s | %(message)s")
        
        self.velocity = 0
        self.heading = 0
        
        self._current_encoder = None
        self._previous_encoder = None

        logging.info("Initialising threads")
        self.threads_init()

        logging.info("Starting all threads")
        self.threads_start()
        
        logging.info("Starting main loop")
        try:
            while True:
                self.calculate_velocity()
                self.calculate_heading()
                print("Heading: " + str(self.heading))
                print("Velocity: " + str(self.velocity))
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logging.fatal(e)
        finally:
            logging.info("Signaling threads to stop")
            self.threads_stop()


    def threads_init(self):
        self.stop_event = Event()

        self.pipe_sensor_imu = deque(maxlen=100)
        self.pipe_sensor_encoder = deque(maxlen=100)
        self.pipe_sensor_stats = deque(maxlen=100)
        self.pipe_sensor_controller = deque(maxlen=100)
        self.pipe_sensor_imageprocessing = deque(maxlen=100)

        self.threads = []
        self.threads.append(IMUThread(self.pipe_sensor_imu, self.stop_event))
        self.threads.append(EncoderThread(self.pipe_sensor_encoder, self.stop_event))
        self.threads.append(StatsThread(self.pipe_sensor_stats, self.stop_event))
        self.threads.append(ControllerThread(self.pipe_sensor_controller, self.stop_event))
        self.threads.append(ImageProcessingThread(self.pipe_sensor_imageprocessing, self.stop_event))
        
    def threads_start(self):
        for thread in self.threads:
            thread.start()

    def threads_stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join()
            
            
    def calculate_velocity(self):
        # 162cm = 20 cycles op as encoder ==> per cycle = 8.1cm, 265 counts per cycle ==> per count = 0.3056603773584905660377358490566cm
        if len(self.pipe_sensor_encoder) > 2:
            self._current_encoder = self.pipe_sensor_encoder[-1]
            if self._previous_encoder != None:
                self.velocity = (self._current_encoder["position"] - self._previous_encoder["position"]) * 0.30566 / (self._current_encoder["time"] - self._previous_encoder["time"]).total_seconds()
            self._previous_encoder = self._current_encoder
        
    def calculate_heading(self):
        if len(self.pipe_sensor_imu) != 0:
            self.heading = self.pipe_sensor_imu[-1]["euler"][0]

            
mcqueen = McQueen()