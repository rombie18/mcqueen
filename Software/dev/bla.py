from threading import *
import time
import random
import queue

import cv2
import numpy as np
import time
import math
import tkinter
import tkinter.messagebox

#Initiate global variables
HSV_dick = {"H_min": 160, "H_max":180, "S_min":100, "S_max":255, "V_min":20, "V_max":255}

def Calibratie_GUI(c):
    main_window = tkinter.Tk()

    frame1 = tkinter.Frame(main_window)
    frame2 = tkinter.Frame(main_window)
    frame3 = tkinter.Frame(main_window)
    frame4 = tkinter.Frame(main_window)
    frame5 = tkinter.Frame(main_window)
    frame6 = tkinter.Frame(main_window)
    frame7 = tkinter.Frame(main_window)
    frame8 = tkinter.Frame(main_window)
    frame9 = tkinter.Frame(main_window)

    lbl_welkom = tkinter.Label(frame1, text="KleurenCalibratie a.d.h.v. sliders voor de HSV waarden.")
    lbl_welkom.pack()


    lbl_H_min = tkinter.Label(frame2, text="H min =")
    slider_H_min = tkinter.Scale(frame2, from_="0", to="180", orient="horizontal", width="30",sliderlength="50",length="500", bd="5", command=slider_H_min_update)
    slider_H_min.set(HSV_dick["H_min"])
    lbl_H_min.pack(side="left")
    slider_H_min.pack(side="left")


    lbl_H_max = tkinter.Label(frame3, text="H max =")
    slider_H_max = tkinter.Scale(frame3, from_="0", to="180", orient="horizontal", width="30",sliderlength="50",length="500", bd="5", command=slider_H_max_update)
    slider_H_max.set(HSV_dick["H_max"])
    lbl_H_max.pack(side="left")
    slider_H_max.pack(side="left")

    lbl_whitespace = tkinter.Label(frame4, text="")
    lbl_whitespace.pack()


    lbl_S_min = tkinter.Label(frame5, text="S min =")
    slider_S_min = tkinter.Scale(frame5, from_="0", to="255", orient="horizontal", width="30",sliderlength="50",length="500", bd="5", command=slider_S_min_update)
    slider_S_min.set(HSV_dick["S_min"])
    lbl_S_min.pack(side="left")
    slider_S_min.pack(side="left")

    lbl_S_max = tkinter.Label(frame6, text="S max =")
    slider_S_max = tkinter.Scale(frame6, from_="0", to="255", orient="horizontal", width="30",sliderlength="50",length="500", bd="5", command=slider_S_max_update)
    slider_S_max.set(HSV_dick["S_max"])
    lbl_S_max.pack(side="left")
    slider_S_max.pack(side="left")

    lbl_whitespace2 = tkinter.Label(frame7, text="")
    lbl_whitespace2.pack()

    lbl_V_min = tkinter.Label(frame8, text="V min =")
    slider_V_min = tkinter.Scale(frame8, from_="0", to="255", orient="horizontal", width="30",sliderlength="50",length="500", bd="5", command=slider_V_min_update)
    slider_V_min.set(HSV_dick["V_min"])
    lbl_V_min.pack(side="left")
    slider_V_min.pack(side="left")

    lbl_V_max = tkinter.Label(frame9, text="V max =")
    slider_V_max = tkinter.Scale(frame9, from_="0", to="255", orient="horizontal", width="30",sliderlength="50",length="500", bd="5", command=slider_V_max_update)
    slider_V_max.set(HSV_dick["V_max"])
    lbl_V_max.pack(side="left")
    slider_V_max.pack(side="left")

    frame1.pack()
    frame2.pack()
    frame3.pack()
    frame4.pack()
    frame5.pack()
    frame6.pack()
    frame7.pack()
    frame8.pack()
    frame9.pack()

    tkinter.mainloop()    

def slider_H_min_update(i):
    HSV_dick["H_min"] = i
    q.put(HSV_dick)
    print(HSV_dick)
def slider_H_max_update(i):
    HSV_dick["H_max"] = i
    q.put(HSV_dick)
    print(HSV_dick)  
def slider_S_min_update(i):
    HSV_dick["S_min"] = i
    q.put(HSV_dick)
    print(HSV_dick)
def slider_S_max_update(i):
    HSV_dick["S_max"] = i
    q.put(HSV_dick)
    print(HSV_dick)
def slider_V_min_update(i):
    HSV_dick["V_min"] = i
    q.put(HSV_dick)
    print(HSV_dick)  
def slider_V_max_update(i):
    HSV_dick["V_max"] = i
    q.put(HSV_dick)
    print(HSV_dick)


 
def Calibratie(c):

    #Getting footage
    webcam_video = cv2.VideoCapture('../images/cocoon.png')

    while True:
        time.sleep(0.1)

        #Specifying upper and lower ranges of color to detect in hsv format
        lower = np.array([160, 100, 20])
        upper = np.array([180, 255, 255]) #These ranges will detect RED

        success, video = webcam_video.read() #Reading footage
        hight, width, _ = video.shape #Get resolution

        video_hsv = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) #Converting BGR image to HSV format

        mask_video = cv2.inRange(video_hsv, lower, upper) # Masking the image to find our color
        
        mask_leftBand = mask_video[0:hight, int(200):int(250)] #Crop right band
        mask_rightBand = mask_video[0:hight, int(width-250):int(width-200)] #crop left band

        mask_contours_leftBand, hierarchy = cv2.findContours(mask_leftBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (leftband)
        mask_contours_rightBand, hierarchy = cv2.findContours(mask_rightBand, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #Finding contours in mask image (rightband)

        # Finding position of all contours of the bands
        if len(mask_contours_leftBand or mask_contours_rightBand) != 0:
            for i_lb, k_rb in zip(mask_contours_leftBand, mask_contours_rightBand):
                if cv2.contourArea(i_lb) > 500:
                    x1, y1, w1, h1 = cv2.boundingRect(i_lb)
                    X_lb = int(x1+(w1/2)) #calculate center of rectangle
                    Y_lb = int(y1+(h1/2))
                    cv2.rectangle(video, (x1+200, y1), (x1 + w1+200, y1 + h1), (0, 0, 255), 3) #drawing rectangle
                
                if cv2.contourArea(k_rb) > 500:
                    x2, y2, w2, h2 = cv2.boundingRect(k_rb)
                    X_rb = int(width-(x2+(w2/2))) #calculate center of rectangle
                    Y_rb = int(y2+(h2/2))
                    cv2.rectangle(video, ((width-x2-250), y2), (((width-x2-250) + w2), (y2 + h2)), (255, 0, 0), 3) #drawing rectangle (formula needed, otherwise it will add it on the left side
            
                

        cv2.imshow("mask image", mask_video) # Displaying mask image
        cv2.imshow("original", video) #Displaying webcam image
        cv2.imshow('Video Left', mask_leftBand) #Displaying left band
        cv2.imshow('Video Right', mask_rightBand) #Displaying right band
        
        cv2.waitKey(1)

if __name__ == '__main__':
    q=queue.Queue()
    t1=Thread(target=Calibratie_GUI, args=(q,))
    t2=Thread(target=Calibratie, args=(q,))
    t1.start()
    t2.start()