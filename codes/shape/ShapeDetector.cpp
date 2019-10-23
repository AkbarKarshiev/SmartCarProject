#include <math.h>
#include <opencv2/opencv.hpp>
#include "ShapeDetector.h"
#include "Object.h"
#include <list>  

using namespace cv;
using namespace std;

void getShapes(Mat src, int thresh);
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
void removeDuplicates(std::list<Object>);
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

ShapeDetector::ShapeDetector()
{};

ShapeDetector::~ShapeDetector()
{};

std::list<Object> ShapeDetector::getShapes(String color, Mat src, int thresh) {
	Mat gray;
	cvtColor(src, gray, CV_BGR2GRAY);
	//convert to gray, blur and apply a threshold

	GaussianBlur(gray, gray, Size(9, 9), 2, 2);
	threshold(gray, gray, thresh, 255, CV_THRESH_BINARY);


	//contours and their hierarchy
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	//find contours and draw them to a third image (contoursImg)
	findContours(gray, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_SIMPLE);// RETR_LIST, CV_CHAIN_APPROX_NONE
	Mat contoursImg = Mat::zeros(src.size(), CV_8UC3);

	for (int i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(125, 125, 125);
		drawContours(contoursImg, contours, i, Scalar(125, 125, 125), 1, 8, hierarchy, 0, Point());
	}

	//approximate contours and write them to result
	Mat result = src.clone();

	//approx will contain the vertices of our objects
	vector<cv::Point> approx;

	objects.clear();
	//convexity hull
	Mat drawing = Mat::zeros(src.size(), CV_8UC3);
		vector<std::vector<int> >hull(contours.size());
		vector<vector<Vec4i>> convDef(contours.size());
		vector<vector<Point>> hull_points(contours.size());
		vector<vector<Point>> defect_points(contours.size());
	

	for (int i = 0; i < contours.size(); i++) {
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

		//skip small ..
		
		if (std::fabs(cv::contourArea(contours[i])) < 1000)
			continue;
		// .. and non-convex shapes except blue signs
		
		if (color == "red" && !cv::isContourConvex(approx))
			continue;
		
		//Triangle
		if (approx.size() == 3 && color == "red") {
			setLabel(result, "Caution", contours[i]);
			cv::Point position = getPosition(contours[i]);
			Object o("Caution", color, position);
			objects.push_back(o);

		}
		//rectangle and hexagon
		else if (( approx.size() >= 4 && approx.size() <= 8)||(approx.size() >= 8 && approx.size() <= 11 && color == "red")) {
			//number ov vertices
			int vertices = approx.size();

			//get degree of all corners
			std::vector<double> cos;
			for (int j = 2; j < vertices + 1; j++) {
				cos.push_back(angle(approx[j % vertices], approx[j - 2], approx[j - 1]));
			}

			//sort ascending the angles
			std::sort(cos.begin(), cos.end());

			//lowest and highest angle?
			double minCos = cos.front();
			double maxCos = cos.back();

			if (vertices == 4 && minCos >= -0.1 && maxCos <= 0.3) {
				cv::Rect rectangle = cv::boundingRect(contours[i]);
				double ratio = std::abs(1 - (double)rectangle.width / rectangle.height);

				//shape detection will recognize the white background as a rectangle, we want to remove it from our collection of shapes
				if (rectangle.size() == result.size())
					continue;

				//check if rectangle has any child shape and color is blue
				if (hierarchy[i][2] != -1 && color == "blue") {
					cv::approxPolyDP(cv::Mat(contours[i + 1]), approx, 10, true);

					//skip small ..
					if (std::fabs(cv::contourArea(contours[i])) < 500)
						continue;
					//if there any triangle
					if (approx.size() == 3) {
						setLabel(result, "Pedestrian", contours[i]);
						cv::Point position = getPosition(contours[i]);
						Object o("Pedestrian", color, position);
						objects.push_back(o);
					}
					//else it is parking
					else {
						setLabel(result, "Parking", contours[i]);
						cv::Point position = getPosition(contours[i]);
						Object o("Parking", color, position);
						objects.push_back(o);

					}
					i++;

				}
				/*else {
					setLabel(result, "RECT", contours[i]);
					cv::Point position = getPosition(contours[i]);
					Object o("RECT", color, position);
					objects.push_back(o);
				}*/

			}
			/*
			else if (vertices == 5 && minCos >= -0.34 && maxCos <= -0.27) {
				setLabel(result, "PENTA", contours[i]);
				cv::Point position = getPosition(contours[i]);
				Object o("PENTA", color, position);
				objects.push_back(o);
			}
			else if (vertices == 6 && minCos >= -0.55 && maxCos <= -0.45) {
				setLabel(result, "HEXA", contours[i]);
				cv::Point position = getPosition(contours[i]);
				Object o("HEXA", color, position);
				objects.push_back(o);
			}*/

			else if (vertices >= 8 && vertices <= 11 && color == "red") {


				setLabel(result, "Stop", contours[i]);
				cv::Point position = getPosition(contours[i]);
				Object o("Stop", color, position);
				objects.push_back(o);
			}
			else if (color == "blue" && vertices == 8) {
					setLabel(result, "Circle-ahead", contours[i]);
					cv::Point position = getPosition(contours[i]);
					Object o("Circle-ahead", color, position);
					objects.push_back(o);
				

			}
			
		}
		else {
			//if the shape isnt a triangle, rectangle or hexagon, it should be a circle
			double area = cv::contourArea(contours[i]);
			cv::Rect rect = cv::boundingRect(contours[i]);
			int radius = rect.width / 2;

			if (std::abs(1 - ((double)rect.width / rect.height)) <= 0.2 &&
				std::abs(1 - (area / (CV_PI * std::pow((float)radius, (float)2)))) <= 0.2) {



				cv::Point position = getPosition(contours[i]);
				/*OLD type
				// tl() directly equals to the desired min. values
				cv::Point minVal = rect.tl();

				// br() is exclusive so we need to subtract 1 to get the max. values
				cv::Point maxVal = rect.br() - cv::Point(1, 1);
				
				bool right = false;
				bool left = false;
				
				//right pixel search
				for (int x = position.x; x < maxVal.x; x++) {
					if (pointPolygonTest(contours[i], Point2f(x, position.y), false) == 0)
					{
						right = true;

					}
				}
				//left pixel search
				for (int x = position.x; x > minVal.x; x--) {
					if (pointPolygonTest(contours[i], Point2f(x, position.y), false) == 0)
					{
						left = true;

					}
				}
				string name = "";
				if (left && right) {
					name = "top";
				}
				else if (right) {
					name = "right";
				}
				else {
					name = "left";
				}
				*/
				string name = "";
				if (!cv::isContourConvex(approx) ) {
					convexHull(contours[i], hull[i], false);
					convexityDefects(contours[i], hull[i], convDef[i]);

					for (int k = 0; k < hull[i].size(); k++)
					{
						int ind = hull[i][k];
						hull_points[i].push_back(contours[i][ind]);
					}
					for (int k = 0; k < convDef[i].size(); k++)
					{
						if (convDef[i][k][3] > 20 * 256) // filter defects by depth
						{
							int ind_0 = convDef[i][k][0];
							int ind_1 = convDef[i][k][1];
							int ind_2 = convDef[i][k][2];
							//cout << "Start::" << contours[i][ind_0] << endl;
							//cout << "DEPTH::" << contours[i][ind_2] << endl;
							//cout << "End::" << contours[i][ind_1] << endl;
							//cout << "Center::" << position << endl;
							int x = contours[i][ind_2].x - position.x;
							int y =  contours[i][ind_2].y - position.y;
							//cout << "x::" << x << endl;
							//cout << "y::" << y << endl;
							
							if (abs(y) > abs(x)) {
								if (y>0) {
									name = "down";
								}
								else {
									name = "top";
								}
							}
							else {
								if (x>0) {
									name = "right";
								}
								else {
									name = "left";
								}
							}
						}
					}

				}
				setLabel(result, name, contours[i]);
				Object o(name, color, position);
				objects.push_back(o);



			}
		}

		//end of loop
	}

	cv::imshow(color, result);

	return objects;
}


/*
* get centered position
*/
Point ShapeDetector::getPosition(vector<Point> contour) {
	cv::Rect r = cv::boundingRect(contour);
	cv::Point pt(r.x + (r.width / 2), r.y + (r.height / 2));
	return pt;
}



/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
double ShapeDetector::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void ShapeDetector::setLabel(cv::Mat & im, const std::string label, std::vector<cv::Point> & contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}


