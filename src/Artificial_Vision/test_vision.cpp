#include "ros/ros.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    Mat image;
    VideoCapture webcam0 = VideoCapture();
    image = imread( "ENSTAVENGERS.png", 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);
    printf("Displayed image.\n");
    waitKey(0);
    return 0;
}
