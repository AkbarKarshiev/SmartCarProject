#ifndef COLORSPLITTER_H
#define COLORSPLITTER_H

#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;

class ColorSplitter {
public:
	ColorSplitter();
	~ColorSplitter();
	Mat getImageChannel(Mat src, String color);
};

#endif