/**
 *@brief Demo code that shows the full functionality of the LaneDetector object.
 */
 /// Command tur run
/// g++ laneDemo4.cpp ./LaneDet/LaneDetector.cpp -o laneDemo -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lpthread -lwiringPi
#include <iostream>
#include <string>
#include <vector>

#include <wiringPi.h>
#include <time.h>
#include "pthread.h"

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp" 
#include <opencv2/highgui/highgui.hpp>
#include <raspicam/raspicam_cv.h>
#include "opencv2/opencv.hpp"

#include "./LaneDet/LaneDetector.hpp"

using namespace cv;
using namespace std;

#define array_size 3
struct struct_laneDet {
    Mat frame;
};

void *laneDetection(void *);

/**
 *@brief Function main that runs the main algorithm of the lane detection.
 *@brief It will read a video of a car in the highway and it will output the
 *@brief same video but with the plotted detected lane
 *@param argv[] is a string to the full path of the demo video
 *@return flag_plot tells if the demo has sucessfully finished
 */
int main(int argc, char *argv[]) {
    struct_laneDet args_Lane;
	pthread_t t4;
	pthread_create(&t4, NULL, laneDetection, (void *)&args_Lane);
        
    while(1){          
       
    } // end of while loop
    pthread_join(t4, NULL);
    return 0;
}

void *laneDetection(void *args_Lane){
    struct struct_laneDet *args_Lane1 = (struct struct_laneDet *)args_Lane;
    short int left_lines = 0, right_lines = 0;
    int i = 0, thr_vp = 20;
    short int isthere_r_l_lines[2] = {0, 0};
    double vanish_arith = 0.0, vanish_x = 0.0;
        
    raspicam::RaspiCam_Cv capture; 
    capture.open();
    
	LaneDetector lanedetector;  // Create the class object
    Mat  img_denoise, greyMat;
    Mat1b img_cannyEdge;
    vector<cv::Vec4i> lines;
    vector<std::vector<cv::Vec4i> > left_right_lines;
    vector<cv::Point> lane;
    
    while (1) {
        Mat copyFrame;
        capture.grab();
        capture.retrieve(args_Lane1 -> frame); //capture orininal video
        resize(args_Lane1 -> frame, args_Lane1 -> frame, Size(320, 240)); // change size of capture
        args_Lane1 -> frame.copyTo(copyFrame);
        cvtColor(copyFrame, greyMat, CV_BGR2GRAY);

        // Get Region of Interest from frame
        Mat RoiBelow = greyMat(Rect(0, 170, 320, 70));
        
        // Denoise image using Gaussian blur
        img_denoise = lanedetector.deNoise(RoiBelow);
        
        // Obtain edges from image using Canny edge detection
        img_cannyEdge = lanedetector.cannyEdge(img_denoise);
        
        // Obtain Hough lines in the cropped image
        lines = lanedetector.houghLines(img_cannyEdge);
        
        if (!lines.empty()) {
            // Separate lines into left and right lines
            left_right_lines = lanedetector.lineSeparation(lines, img_cannyEdge);
            
            // Apply regression to obtain only one line for each side of the lane
            lane = lanedetector.regression(left_right_lines, args_Lane1 -> frame);
            
            if (left_right_lines[0].empty()) // right lines
            {
                isthere_r_l_lines[0] = 0;
            } else isthere_r_l_lines[0] = 1;
            
             if (left_right_lines[1].empty()) // left lines
            {
                isthere_r_l_lines[1] = 0;
            } else isthere_r_l_lines[1] = 1;
            
            // Get vanishing point of the the lines
            vanish_x = lanedetector.getVanish();
            
            vanish_arith += vanish_x;
            left_lines += isthere_r_l_lines[1];
            right_lines += isthere_r_l_lines[0];
            
            // Predict the turn by determining the vanishing point of the the lines
            if ((i+1)%array_size == 0)
            {              
                vanish_arith = vanish_arith/array_size;
                
                if(left_lines >= 3 && right_lines >= 3){
                    if(vanish_arith < (160 - thr_vp)){
                        cout << "Left turn: 20, 60" << endl;
                    } 
                    else if(vanish_arith > (160 + thr_vp)){
                        cout << "Right turn: 20, 60" << endl;
                    } 
                    else if(vanish_arith >= (160 - thr_vp) && vanish_arith <= (160 + thr_vp)){
                        cout << "Forward" << endl;
                    }
                } 
                else if (left_lines < 3 || right_lines < 3){
                    if(vanish_arith < (160 - thr_vp) && vanish_arith > 65){
                        cout << "Left 90 turn: 20, 75" << endl;
                    }
                    else if(vanish_arith > (160 + thr_vp) && vanish_arith < 265){
                        cout << "Right 90 turn: 75, 20" << endl;
                    } 
                }
                
                left_lines = 0; 
                right_lines = 0;
                vanish_arith = 0;
            }
            
            // Plot vanishing point value detection
            
                std::ostringstream strs;
                strs << vanish_x;
                std::string str = strs.str();
                
                cv::putText(img_cannyEdge, str, cv::Point(80, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(255, 255, 255), 1, CV_AA);
               
                cv::imshow("vanish", img_cannyEdge);  
            // end of Plot vanish point
            
            } else {
                cout << "No Lines Detected yet" << endl;
        }
        
        i = (i+1)%array_size;
        
        if(cv::waitKey(25) == 'q')
            break;
    }
    pthread_exit(NULL);
}








