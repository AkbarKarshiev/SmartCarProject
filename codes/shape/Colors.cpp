#include <math.h>
#include "Colors.h"
#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;

//constructor
Colors::Colors()
{};


Colors::~Colors()
{};

Mat Colors::getImageChannel(Mat src, String color) {
	Mat hsv;

	if (color.compare("blue") == 0) {
		/*
		*BLUE
		*/
		Mat mask_blue;
		Mat blue(src.size().height, src.size().width, CV_8UC3, Scalar(255, 255, 255));			
		//convert to HSV
		cvtColor(src, hsv, CV_BGR2HSV);
		//define color range in HSV
		Scalar lowerBlue = Scalar(55, 50, 50);
		Scalar upperBlue = Scalar(130, 255, 255);
		//Threshold the HSV image to get a mask
		inRange(hsv, lowerBlue, upperBlue, mask_blue);
		//apply mask
		src.copyTo(blue, mask_blue);
		//call function
		return blue;
	} else if (color.compare("red") == 0) {
		/*
		*RED
		*/
		Mat mask_red;
		Mat mask_red1;
		Mat mask_red2;
		Mat red(src.size().height, src.size().width, CV_8UC3, Scalar(255, 255, 255));
		//convert to HSV
		cvtColor(src, hsv, CV_BGR2HSV);
		//define color range in HSV
		Scalar lowerRed1 = Scalar(0, 50, 50);
		Scalar upperRed1 = Scalar(15, 255, 255);
		Scalar lowerRed2 = Scalar(150, 50, 50);
		Scalar upperRed2 = Scalar(179, 255, 255);
		//Threshold the HSV image to get a mask
		inRange(hsv, lowerRed1, upperRed1, mask_red1);
		inRange(hsv, lowerRed2, upperRed2, mask_red2);
		bitwise_or(mask_red1, mask_red2, mask_red);
		//apply mask
		src.copyTo(red, mask_red);
		//call function
		return red;
	} else if (color.compare("green") == 0) {
		/*
		*GREEN
		*/
		Mat mask_green;
		Mat green(src.size().height, src.size().width, CV_8UC3, Scalar(255, 255, 255));
		//convert to HSV
		cvtColor(src, hsv, CV_BGR2HSV);
		//define color range in HSV
		Scalar lowerGreen = Scalar(40, 50, 50);
		Scalar upperGreen = Scalar(84, 255, 255);
		//Threshold the HSV image to get a mask
		inRange(hsv, lowerGreen, upperGreen, mask_green);
		//apply mask
		src.copyTo(green, mask_green);
		//call function
		return green;
	}

}