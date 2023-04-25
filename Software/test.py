import cv2  # Import the OpenCV library to enable computer vision
import numpy
import edge_detection as edge  # Handles the detection of lane lines
import matplotlib.pyplot as plt  # Used for plotting and error checking

original_frame = cv2.imread('foto.pdf',0)
cv2.imshow('image', original_frame)