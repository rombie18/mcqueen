#Importing all modules
import cv2
import numpy as np
import time
import math

X_lb=0
Y_lb=0
X_rb=0
Y_rb=0

h1= 0
h2= 0



#Specifying upper and lower ranges of color to detect in hsv format
lower = np.array([160, 100, 20])
upper = np.array([180, 255, 255]) #These ranges will detect RED

#Getting footage
webcam_video = cv2.VideoCapture(0)

while True:
    time.sleep(0.1)
    success, video = webcam_video.read() #Reading footage
    hight, width, _ = video.shape #Get resolution

    video_hsv = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) #Converting BGR image to HSV format

    mask_video = cv2.inRange(video_hsv, lower, upper) # Masking the image to find our color
    
    mask_leftBand = mask_video[0:hight, int(0):int(50)] #Crop right band
    mask_rightBand = mask_video[0:hight, int(width-50):width] #crop left band

    mask_contours_leftBand, hierarchy = cv2.findContours(mask_leftBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (leftband)
    mask_contours_rightBand, hierarchy = cv2.findContours(mask_rightBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (rightband)

    # Finding position of all contours of the bands
    if len(mask_contours_leftBand or mask_contours_rightBand) != 0:
        for i_lb, k_rb in zip(mask_contours_leftBand, mask_contours_rightBand):
            if cv2.contourArea(i_lb) > 500:
                x1, y1, w1, h1 = cv2.boundingRect(i_lb)
                X_lb = int(x1+(w1/2)) #calculate center of rectangle
                Y_lb = int(y1+(h1/2))
            
            if cv2.contourArea(k_rb) > 500:
                x2, y2, w2, h2 = cv2.boundingRect(k_rb)
                X_rb = int(width-(x2+(w2/2))) #calculate center of rectangle
                Y_rb = int(y2+(h2/2)) 
            
            #Calculate per frame how the car should steer
            diff = 1
            if h2 != 0:
                diff = h1/h2
            if (diff > 0.8) and (diff < 1.2):
                print("Go straight")
            if diff > 1.2:
                i = 89.9133*math.exp(0.001*diff)
                print("Go right, with angle:", i)
            if diff < 0.8:
                j = 89.9376*math.exp(0.6938*diff)
                print("Go left, with angle:", j)
            

    cv2.waitKey(1)
