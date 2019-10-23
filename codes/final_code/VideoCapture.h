#pragma once
#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;

class VideoCapture {
public:
	VideoCapture();
	~VideoCapture();
	void openVideo(String path);
	void openImage(String path);
	cv::Mat openCamera(int channel);
};

#endif