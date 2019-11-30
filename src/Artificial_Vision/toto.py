import numpy as np
import cv2
from chessboard_calibration import getCamDistortData


if __name__ == "__main__":
    mtx, dist = getCamDistortData('calibration_data.txt')

    img = cv2.imread('calibration_images/fra69.png')


    # undistort
    undistorded = cv2.undistort(img, mtx, dist, img, mtx)


    cv2.imwrite('calibration_images/calibresult_fra69_py2.png',undistorded)


