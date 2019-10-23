#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <ctype.h>
#include <time.h> 

#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video\tracking.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace cv;
using namespace std;

// Detect Skin from YCrCb
Mat DetectYCrCb(Mat img, Scalar min, Scalar max) {
	Mat skin;
	cvtColor(img, skin, cv::COLOR_BGR2YCrCb);
	inRange(skin, min, max, skin);
	Mat rect_12 = getStructuringElement(cv::MORPH_RECT, Size(12, 12), Point(6, 6));
	erode(skin, skin, rect_12, Point(), 1);
	Mat rect_6 = getStructuringElement(cv::MORPH_RECT, Size(6, 6), Point(3, 3));
	dilate(skin, skin, rect_6, Point(), 2);
	return skin;
}

void DetectContour(Mat img) {
	Mat drawing = Mat::zeros(img.size(), CV_8UC3);
	vector<vector<Point> > contours;
	vector<vector<Point> > bigContours;
	vector<Vec4i> hierarchy;

	findContours(img, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, Point());

	if (contours.size() > 0)
	{
		vector<std::vector<int> >hull(contours.size());
		vector<vector<Vec4i>> convDef(contours.size());
		vector<vector<Point>> hull_points(contours.size());
		vector<vector<Point>> defect_points(contours.size());


		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > 5000)
			{
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
						defect_points[i].push_back(contours[i][ind_2]);
						cout << "Start::"<<contours[i][ind_0]<<endl;
						cout << "End::"<<contours[i][ind_1]<<endl;
						cv::circle(drawing, contours[i][ind_0], 5, Scalar(0, 255, 0), -1);
						cv::circle(drawing, contours[i][ind_1], 5, Scalar(0, 255, 0), -1);
						cv::circle(drawing, contours[i][ind_2], 5, Scalar(0, 0, 255), -1);
						cv::line(drawing, contours[i][ind_2], contours[i][ind_0], Scalar(0, 0, 255), 1);
						cv::line(drawing, contours[i][ind_2], contours[i][ind_1], Scalar(0, 0, 255), 1);
					}
				}

				drawContours(drawing, contours, i, Scalar(0, 255, 0), 1, 8, vector<Vec4i>(), 0, Point());
				drawContours(drawing, hull_points, i, Scalar(255, 0, 0), 1, 8, vector<Vec4i>(), 0, Point());
			}
		}
	}
	imshow("Hull demo", drawing);
}


int main(int argc, char** argv)
{
	String imageDirectory = "left.png";
	Mat frame, copyFrame;

	namedWindow("Hull demo", cv::WINDOW_AUTOSIZE);
	
	//IMAGE
		

		// Read the file
	frame = imread(imageDirectory, 1);

	// Check for invalid input
	if (!frame.data)
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}


	while (true) {
		//detect shapes
		Mat skinYCrCb = DetectYCrCb(frame, Scalar(0, 100, 80), Scalar(255, 185, 135));

		DetectContour(skinYCrCb);

		int c = waitKey(10);
		if ((char)c == 27)
		{
			break;
		}
	}
	
	cv::destroyAllWindows();
	return 0;
}