#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# import the necessary packages
import cv2
import numpy as np
from time import time
import socket
from goprocam import GoProCamera, constants

from chessboard_calibration import getCamDistortData


### Initialisation
gpCam = GoProCamera.GoPro()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time()

res, fps = '720p', 120

gpCam.streamSettings("250000", "0")
gpCam.video_settings(res=res, fps=str(fps))
gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
gpCam.livestream("start")

cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
c = 0

# Distortion data
# Read the camera matrix from calibration file
calibration_matrix, calibration_dist = getCamDistortData('calibration_data.txt')


cv2.namedWindow('Original frame', cv2.WINDOW_NORMAL)
cv2.namedWindow('Undistorded frame', cv2.WINDOW_NORMAL)

while True:
    nmat, image = cap.read()
    print(np.shape(image))
    #show the frame
    cv2.imshow('Original frame',image)
    print("Original size: ", np.shape(image))
    
    undistorded = cv2.undistort(image, calibration_matrix, calibration_dist, None)
    cv2.imshow('Undistorded frame',undistorded)
    print("Undistorded size: ", np.shape(undistorded))
    
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
gpCam.power_off()


