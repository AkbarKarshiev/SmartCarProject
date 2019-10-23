/**
 *@brief Header file for the LaneDetector class. Functions are developed in LaneDetector.cpp
 */

#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

/**
 *@brief Definition of the LaneDetector class. It contains all the functions and variables depicted in the
 *@brief Activity diagram and UML Class diagram.
 *@brief It detects the lanes in an image if a highway and outputs the
 *@brief same image with the plotted lane.
 */
class LaneDetector {
private:
    double img_center;
    bool left_flag = false;  // Tells us if there's left boundary of lane detected
    bool right_flag = false;  // Tells us if there's right boundary of lane detected
    cv::Point right_b;  // Members of both line equations of the lane boundaries:
    double right_m;  // y = m*x + b
    cv::Point left_b;  //
    double left_m;  //
    double max_vanish = 0;
    double min_vanish = 255;
public:
    ///cv::Mat getYellowImg(cv::Mat originalImg); // Get yellow colored image from original
    cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
    cv::Mat cannyEdge(cv::Mat originalImg); // Canny edge detection
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);  // Detect Hough lines in masked edges image
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);  // Sprt detected lines by their slope into right and left lines
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);  // Get only one line for each side of the lane
    double getVanish();  // Determine vanishing point end return it
};
