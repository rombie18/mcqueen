import sys
sys.path.append("../libs/lane_detection")

import cv2
import json
from lane import Lane
import time
import math
import numpy as np

cap = cv2.VideoCapture('../main/snap.png')

while cap.isOpened():
    # Load a frame (or image)
    _, original_frame = cap.read()   
    
    # Discard unwanted part of image
    #original_height, original_width, original_channels = original_frame.shape
    #cropped_frame = original_frame[280:original_height, 0:original_width]
    
    # Create a Lane object
    lane_obj = Lane(orig_frame=original_frame)
    
    # Perform thresholding to isolate lane lines
    lane_line_markings = lane_obj.get_line_markings()
    # If there are less than 50 'lane' pixels detected, skip calculations
    #if numpy.sum(lane_line_markings == 255) < 50:
    #    continue
    
    # Plot the region of interest on the image
    lane_obj.plot_roi(plot=True)
    
    # Perform the perspective transform to generate a bird's eye view
    # If Plot == True, show image with new region of interest
    warped_frame = lane_obj.perspective_transform(plot=True)
    
    # 
    if np.sum(warped_frame == 255) < 3000:
        print("Only {} valid pixels, skipping frame".format(np.sum(warped_frame == 255)))
        continue
    
    # Generate the image histogram to serve as a starting point
    # for finding lane line pixels
    histogram = lane_obj.calculate_histogram(plot=True)  
        
    # Find lane line pixels using the sliding window method 
    left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
        plot=True)
    
    # Fill in the lane line
    lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=True)
        
    # Overlay lines on the original frame
    frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=True)
    
    # Calculate lane line curvature (left and right lane lines)
    lane_obj.calculate_curvature(print_to_terminal=True)
    
    # Calculate center offset                                                                 
    lane_obj.calculate_car_position(print_to_terminal=True)

    # Calculate turning angle
    wheel_base = 0.3
    curvem = (lane_obj.left_curvem + lane_obj.right_curvem) / 2
    angle = math.degrees(math.atan(wheel_base / curvem)) * 6 + 90

    print(angle)
    
    cv2.imshow('frame', frame_with_lane_lines)
    if cv2.waitKey(1) == ord('q'):
        break