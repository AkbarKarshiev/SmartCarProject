#ifndef COLORS_H
#define COLORS_H

#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;

class Colors {
public:
	Colors();
	~Colors();
	Mat getImageChannel(Mat src, String color);
};

#endif