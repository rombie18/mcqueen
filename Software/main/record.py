#!/usr/bin/env python3

# Copyright 2019 The Imaging Source Europe GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# This example will show you how to save a video stream to a file
#

import json
import time
import sys
import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst

def block_until_playing(pipeline):

    while True:
        # wait 0.1 seconds for something to happen
        change_return, state, pending = pipeline.get_state(100000000)
        if change_return == Gst.StateChangeReturn.SUCCESS:
            return True
        elif change_return == Gst.StateChangeReturn.FAILURE:
            print("Failed to change state {} {} {}".format(change_return,
                                                           state,
                                                           pending))
            return False

def main():

    Gst.init(sys.argv)  # init gstreamer

    # this line sets the gstreamer default logging level
    # it can be removed in normal applications
    # gstreamer logging can contain verry useful information
    # when debugging your application
    # see https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html
    # for further details
    Gst.debug_set_default_threshold(Gst.DebugLevel.WARNING)

    serial = "02320237"

    #pipeline = Gst.parse_launch("tcambin name=bin"
    #                            " ! video/x-raw,format=BGRx,width=640,height=480,framerate=30/1"
    #                            " ! tee name=t"
    #                            " ! queue"
    #                            " ! videoconvert"
    #                            " ! ximagesink"
    #                            " t."
    #                            " ! queue"
    #                            " ! videoconvert"
    #                            " ! avimux"
    #                            " ! filesink name=fsink")

    # to save a video without live view reduce the pipeline to the following:

    pipeline = Gst.parse_launch("tcambin name=bin"
                                " ! video/x-raw,format=BGRx,width=1280,height=720,framerate=60/1"
                                " ! videoconvert"
                                " ! avimux"
                                " ! filesink name=fsink")

    # serial is defined, thus make the source open that device
    if serial is not None:
        camera = pipeline.get_by_name("bin")
        camera.set_property("conversion-element", "tcamdutils-cuda")
        camera.set_property("serial", serial)

    file_location = "/tmp/tiscamera-save-stream.avi"

    fsink = pipeline.get_by_name("fsink")
    fsink.set_property("location", file_location)

    pipeline.set_state(Gst.State.PLAYING)

    if not block_until_playing(pipeline):
        print("Unable to start pipeline")

    new_state = Gst.Structure.new_empty("tcam")

    with open('camera_properties.json', 'r') as file:
        data = json.load(file)

        for key, item in data.items():
            new_state.set_value(key, item)

    # this can also be done by calling Gst.Structure.from_string()

    camera.set_property("tcam-properties", new_state)

    print("Press Ctrl-C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
