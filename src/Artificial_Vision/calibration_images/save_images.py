#!/usr/bin/env python3
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
import cv2
import numpy as np
from time import time
import socket
from goprocam import GoProCamera, constants



### Initialisation
gpCam = GoProCamera.GoPro()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time()
gpCam.livestream("start")
gpCam.video_settings(res='720p', fps='30')
gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)

c = 33


while True:
    nmat, image = cap.read()
    
    #show the frame
    cv2.imshow('Webcam',image)
    
    # Keep awaken
    if time() - t >= 2.5:
        sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
        t=time()
    

    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
        break

    # if the SPACE key was pressed, take a picture
    elif key == 32:
        c += 1
        cv2.imwrite('fra%i.png'%c,image)
        print("Picture saved")
        
    
# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
