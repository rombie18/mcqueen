import sys
sys.path.append("../libs")

import time
import json
import sys

from tis import TIS


# Image processing
print("Initialising image processing...")
Tis = TIS.TIS()
Tis.open_device("02320237", 1280, 720, "60/1", TIS.SinkFormats.BGRA, True)
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