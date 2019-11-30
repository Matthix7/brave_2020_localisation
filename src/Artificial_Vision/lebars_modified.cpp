// OpenCV headers.
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


using namespace cv;

// Need to send first : http://10.5.5.9/gp/gpControl/execute?p1=gpStream&a1=proto_v2&c1=restart
// See GoProStream.py
// Keep-alive command : "_GPHD_:0:0:2:0.000000"



int main(int argc, char* argv[])
{
	// Will store the code of the keyboard key pressed to terminate the program.
	int c = 0;

  printf("Connecting...\n");
	 //Keep-alive...
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons((unsigned short)atoi("8554"));
	sa.sin_addr.s_addr = inet_addr("10.5.5.9");
	int salen = sizeof(sa);
  printf("Connection setup completed.\n");
	
  
  // Open camera number 0.
  
  VideoCapture* webcam0;
  webcam0 = new VideoCapture("udp://@10.5.5.9:8554");
  
	
//  cv::VideoCapture webcam0 = cv::VideoCapture();
//	int toto = webcam0.open("udp://10.5.5.9:8554", cv::CAP_FFMPEG);
  
  printf("Camera open: %d.\n", webcam0->isOpened());
  
  
  
	// Set desired camera resolution. Depending on the camera, some settings might not be available.
	webcam0->set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	webcam0->set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  
//  webcam0.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//	webcam0.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  
  printf("Image size set.\n");
  
  
	return 0;
}
