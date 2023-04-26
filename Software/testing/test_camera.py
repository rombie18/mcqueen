import sys
sys.path.append("/home/mcqueen/mcqueen/Software/libs/tis")

import time
import json

from tis import TIS, SinkFormats


# Image processing
print("Initialising image processing...")
Tis = TIS()
Tis.open_device("02320237", 1280, 720, "60/1", SinkFormats.BGRA, True)
Tis.start_pipeline()

with open('camera_properties.json', 'r') as file:
    data = json.load(file)
    for key, item in data.items():
        try:
            Tis.set_property(key, item)
        except Exception as error:
            pass

while True:
    time.sleep(0.1)