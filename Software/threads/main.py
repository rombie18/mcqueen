import time
import logging
import Jetson.GPIO as GPIO
from collections import deque
from threading import Event
from customthreads import IMUThread, EncoderThread, StatsThread, ControllerThread, ImageProcessingThread, DataCollectionThread

class McQueen:
    def __init__(self):
        
        logging.getLogger()
        logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(threadName)s | %(message)s")
        
        self.flag_initialised = False
        self.flag_fault = False
        self.flag_halt = False
        self.flag_stop = False
        
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
                self.process_calculations()
                self.process_flags()
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logging.fatal(e)
        finally:
            logging.info("Signaling threads to stop")
            self.threads_stop()
            GPIO.cleanup()


    ### Threading ###

    def threads_init(self):
        self.stop_event = Event()
        
        self.init_events = []
        self.init_event_imu = Event()
        self.init_event_encoder = Event()
        self.init_event_stats = Event()
        self.init_event_controller = Event()
        self.init_event_imageprocessing = Event()
        self.init_event_datacollection = Event()
        self.init_events.append(self.init_event_imu)
        self.init_events.append(self.init_event_encoder)
        self.init_events.append(self.init_event_stats)
        self.init_events.append(self.init_event_controller)
        self.init_events.append(self.init_event_imageprocessing)
        self.init_events.append(self.init_event_datacollection)

        self.pipe_imu = deque(maxlen=1000)
        self.pipe_encoder = deque(maxlen=1000)
        self.pipe_stats = deque(maxlen=1000)
        self.pipe_controller = deque(maxlen=1000)
        self.pipe_imageprocessing = deque(maxlen=1000)

        self.pipes = {
            'imu': self.pipe_imu,
            'encoder': self.pipe_encoder,
            'stats': self.pipe_stats,
            'controller': self.pipe_controller,
            'imageprocessing': self.pipe_imageprocessing,
        }

        self.threads = []
        self.threads.append(IMUThread(self.pipe_imu, self.stop_event, self.init_event_imu))
        self.threads.append(EncoderThread(self.pipe_encoder, self.stop_event, self.init_event_encoder))
        self.threads.append(StatsThread(self.pipe_stats, self.stop_event, self.init_event_stats))
        self.threads.append(ControllerThread(self.pipe_controller, self.stop_event, self.init_event_controller))
        self.threads.append(ImageProcessingThread(self.pipe_imageprocessing, self.stop_event, self.init_event_imageprocessing))
        self.threads.append(DataCollectionThread(None, self.stop_event, self.init_event_datacollection, self.pipes))
        
    def threads_start(self):
        for thread in self.threads:
            thread.start()

    def threads_stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join()
            
            
    ### Calculations ###
    def process_calculations(self):
        self.__calculate_velocity()
        self.__calculate_heading()
            
    def __calculate_velocity(self):
        # 162cm = 20 cycles op as encoder ==> per cycle = 8.1cm, 265 counts per cycle ==> per count = 0.3056603773584905660377358490566cm
        if len(self.pipe_encoder) > 2:
            self._current_encoder = self.pipe_encoder[-1]
            if self._previous_encoder != None:
                self.velocity = -(self._current_encoder["position"] - self._previous_encoder["position"]) * 0.30566 / (self._current_encoder["time"] - self._previous_encoder["time"]).total_seconds()
            self._previous_encoder = self._current_encoder
        
    def __calculate_heading(self):
        if len(self.pipe_imu) != 0:
            self.heading = self.pipe_imu[-1]["euler"][0]
            
            
    ### Flags ###
    def process_flags(self):
        self.__flag_initialised()

    def __flag_initialised(self):
        for init_event in self.init_events:
            if not init_event.is_set():
                self.flag_initialised = False
                return
        self.flag_initialised = True

            
mcqueen = McQueen()