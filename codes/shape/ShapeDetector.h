#ifndef SHAPEDETECTOR_H
#define SHAPEDETECTOR_H

#include "opencv2/opencv.hpp" 
#include <vector>
#include <list>
#include "Object.h"

using namespace cv;
using namespace std;

class ShapeDetector {
public:
	ShapeDetector();
	~ShapeDetector();
	std::list<Object> getShapes(String name, Mat src, int thresh);
	void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	cv::Point getPosition(vector<Point> contour);
	//string detectShapes(vector<cv::Point> approx, vector<Point> contour, Size result, int hierarchy);
private:
	
	std::list<Object> objects;
};

#endif