import sys
sys.path.append("/home/mcqueen/mcqueen/Software/libs/lane_detection")
sys.path.append("/home/mcqueen/mcqueen/Software/libs/tis")

import cv2
import json
from lane import Lane
from tis import TIS, SinkFormats
import time

camera = TIS()
camera.open_device("02320237", 1280, 720, "60/1", SinkFormats.BGRA, False)

with open('/home/mcqueen/mcqueen/Software/main/camera_properties.json', 'r') as file:
    data = json.load(file)
    for key, item in data.items():
        camera.set_property(key, item)

try:
    camera.set_property("TriggerMode","Off")
except Exception as error:
    print(error)

camera.start_pipeline()

while True:
    
    if camera.snap_image(1):                                        
        # Load a frame (or image)
        original_frame = camera.get_image()
        
        # Discard unwanted part of image
        original_height, original_width, original_channels = original_frame.shape
        cropped_frame = original_frame[280:original_height, 0:original_width]
        
        # Create a Lane object
        lane_obj = Lane(orig_frame=cropped_frame)
        
        # Perform thresholding to isolate lane lines
        lane_line_markings = lane_obj.get_line_markings()
        # If there are less than 50 'lane' pixels detected, skip calculations
        #if numpy.sum(lane_line_markings == 255) < 50:
        #    continue
        
        # Plot the region of interest on the image
        lane_obj.plot_roi(plot=False)
        
        # Perform the perspective transform to generate a bird's eye view
        # If Plot == True, show image with new region of interest
        warped_frame = lane_obj.perspective_transform(plot=False)
        
        # Generate the image histogram to serve as a starting point
        # for finding lane line pixels
        histogram = lane_obj.calculate_histogram(plot=False)  
            
        # Find lane line pixels using the sliding window method 
        left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
            plot=False)
        
        # Fill in the lane line
        lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
            
        # Overlay lines on the original frame
        frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
        
        # Calculate lane line curvature (left and right lane lines)
        lane_obj.calculate_curvature(print_to_terminal=True)
        
        # Calculate center offset                                                                 
        lane_obj.calculate_car_position(print_to_terminal=True)

        cv2.imshow('frame', frame_with_lane_lines)
        if cv2.waitKey(1) == ord('q'):
            break