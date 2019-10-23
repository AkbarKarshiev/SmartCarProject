#include <iostream>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp" 
#include <opencv2/highgui/highgui.hpp>
//#include <raspicam/raspicam_cv.h>
#include "opencv2/opencv.hpp"
#include "Colors.h"
#include "ShapeDetector.h"
#include "CollectionAggregator.h"
#include "Object.h"
#include <time.h>
/// g++ main.cpp ShapeDetector.cpp ColorSplitter.cpp CollectionAggregator.cpp Object.cpp -o shape -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv`

using namespace cv;
using namespace std;

int thresh = 150;

Colors colors;
ShapeDetector shapeDetector;
CollectionAggregator aggregator;
std::list<Object> objects;
std::list<Object> result;



void findShapes(Mat src);


String imageDirectory = "test.png"; // should contain ==> path/image_name

bool isImage = true;  // if you diceded to use image; otherwise ==> false

RNG rng(120);

int main(){
	Mat src;
	if(!isImage){
		/*
		*Camera VIDEO
		*/
	   //  --- For Windows recomment here ---
		VideoCapture cap(0);
		namedWindow("src", WINDOW_AUTOSIZE);
		for (;;){
			//Load frame
			cap >> src;
			//detect shapes
			findShapes(src);

			if (waitKey(30) == 27) break;
		}
		
		
		/* 
		// --- For Raspberry recomment here ---
		raspicam::RaspiCam_Cv capture;
		capture.open();
		Mat src;
		while (1) {

			capture.grab();
			capture.retrieve(src); //capture orininal video
			resize(src, src, Size(320, 240)); // change size of capture

			// Get Region of Interest from frame
			//Mat RoiBelow = frame(Rect(0, 150, 320, 90));


			findShapes(src);
			if (cv::waitKey(25) == 'q')
				break;
		}
		*/
	}
	else{
		/*
		*IMAGE
		*/

		// Read the file
		src = imread(imageDirectory, 1);

		// Check for invalid input
		if (!src.data)
		{
			cout << "Could not open or find the image" << std::endl;
			return -1;
		}


		while (true){
			//detect shapes
			findShapes(src);

			if (waitKey(50) == 27) break;
		}
	}

	return 0;
}



void findShapes(Mat src){
	Mat blue = colors.getImageChannel(src, "blue");
	objects = shapeDetector.getShapes("blue", blue, thresh);
	aggregator.append(objects);

	Mat red = colors.getImageChannel(src, "red");
	objects = shapeDetector.getShapes("red", red, thresh);
	aggregator.append(objects);
	
	
	
	//output
	cout << "------" << endl;
	result = aggregator.retrieve();

	list<Object>::iterator iter = result.begin();

	while (iter != result.end()) {
		Object& temp = *iter;
		
		cout << temp.getName()<<endl;
		
		iter++;
	}


	aggregator.setNewCycle();


	imshow("src", src);
}


