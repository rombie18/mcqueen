import sys
sys.path.append("../libs/lane_detection")

import cv2
import json
from lane import Lane
import edge_detection as edge # Handles the detection of lane lines
import time
import math
import numpy as np

cap = cv2.VideoCapture('../videos/zijkant.mp4')

while cap.isOpened():
    # Load a frame (or image)
    _, original_frame = cap.read()   
    
    # Discard unwanted part of image
    original_height, original_width, original_channels = original_frame.shape
    frame = original_frame[400:original_height, 0:original_width]
                        
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    hls = edge.blur_gaussian(hls, ksize=11) # Reduce noise
    
    ################### Isolate possible lane line edges ######################
        
    # Perform Sobel edge detection on the L (lightness) channel of 
    # the image to detect sharp discontinuities in the pixel intensities 
    # along the x and y axis of the video frame.             
    # sxbinary is a matrix full of 0s (black) and 255 (white) intensity values
    # Relatively light pixels get made white. Dark pixels get made black.
    _, sxbinary = edge.threshold(hls[:, :, 1], thresh=(120, 255))
    #sxbinary = edge.blur_gaussian(sxbinary, ksize=31) # Reduce noise

    # 1s will be in the cells with the highest Sobel derivative values
    # (i.e. strongest lane line edges)
    sxbinary = edge.mag_thresh(sxbinary, sobel_kernel=3, thresh=(110, 255))

    ######################## Isolate possible lane lines ######################

    # Perform binary thresholding on the S (saturation) channel 
    # of the video frame. A high saturation value means the hue color is pure.
    # We expect lane lines to be nice, pure colors (i.e. solid white, yellow)
    # and have high saturation channel values.
    # s_binary is matrix full of 0s (black) and 255 (white) intensity values
    # White in the regions with the purest hue colors (e.g. >80...play with
    # this value for best results).
    s_channel = hls[:, :, 2] # use only the saturation channel data
    _, s_binary = edge.threshold(s_channel, (100, 255))
    
    # Perform binary thresholding on the R (red) channel of the 
        # original BGR video frame. 
    # r_thresh is a matrix full of 0s (black) and 255 (white) intensity values
    # White in the regions with the richest red channel values (e.g. >120).
    # Remember, pure white is bgr(255, 255, 255).
    # Pure yellow is bgr(0, 255, 255). Both have high red channel values.
    _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(100, 255))
    _, g_thresh = edge.threshold(frame[:, :, 1], thresh=(100, 255))
    _, b_thresh = edge.threshold(frame[:, :, 0], thresh=(100, 255))

    # Lane lines should be pure in color and have high red channel values 
    # Bitwise AND operation to reduce noise and black-out any pixels that
    # don't appear to be nice, pure, solid colors (like white or yellow lane 
    # lines.)
    
    # Keep parts of images that are in range r_thresh and out of range g_thresh and b_thresh
    rs_binary = cv2.bitwise_and(s_binary, cv2.bitwise_and(r_thresh, cv2.bitwise_not(cv2.bitwise_and(g_thresh, b_thresh))))

    ### Combine the possible lane lines with the possible lane line edges ##### 
    # If you show rs_binary visually, you'll see that it is not that different 
    # from this return value. The edges of lane lines are thin lines of pixels.
    lane_line_markings = cv2.bitwise_or(rs_binary, sxbinary.astype(
                            np.uint8))
    
    kernel = np.ones((1,51),np.uint8)
    closing = cv2.morphologyEx(lane_line_markings, cv2.MORPH_CLOSE, kernel)
    
    # Generate the histogram
    histogram = np.sum(frame[int(frame.shape[0]/2):,:], axis=0)
    peak = np.argmax(histogram)
        
    # Draw both the image and the histogram
    ax2.plot(histogram)
    ax2.set_title("Histogram Peaks")
    plt.show()
    
    cv2.imshow('frame', closing)
    if cv2.waitKey(1) == ord('q'):
        break
    
    time.sleep(0.1)