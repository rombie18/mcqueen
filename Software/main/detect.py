import sys
sys.path.append("../libs")

import cv2

from lane_detection.lane import Lane

filename = 'color_corr.png'

def main():
     
  # Load a frame (or image)
  original_frame = cv2.imread(filename)
  
  # Discard unwanted part of image
  original_height, original_width, original_channels = original_frame.shape
  cropped_frame = original_frame[280:original_height, 0:original_width]
 
  # Create a Lane object
  lane_obj = Lane(orig_frame=cropped_frame)
 
  # Perform thresholding to isolate lane lines
  lane_line_markings = lane_obj.get_line_markings()
 
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
    plot=True)
 
  # Fill in the lane line
  lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
     
  # Overlay lines on the original frame
  frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
 
  # Calculate lane line curvature (left and right lane lines)
  lane_obj.calculate_curvature(print_to_terminal=False)
 
  # Calculate center offset                                                                 
  lane_obj.calculate_car_position(print_to_terminal=False)
     
  # Display curvature and center offset on image
  frame_with_lane_lines2 = lane_obj.display_curvature_offset(
    frame=frame_with_lane_lines, plot=True)
     
  # Create the output file name by removing the '.jpg' part
  size = len(filename)
  new_filename = filename[:size - 4]
  new_filename = new_filename + '_thresholded.jpg'     
     
  # Save the new image in the working directory
  #cv2.imwrite(new_filename, lane_line_markings)
 
  # Display the image 
  #cv2.imshow("Image", lane_line_markings)
     
  # Display the window until any key is pressed
  cv2.waitKey(0) 
     
  # Close all windows
  cv2.destroyAllWindows() 
     
main()