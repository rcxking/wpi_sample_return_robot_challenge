#ifndef __IO_H_INCLUDED__
#define __IO_H_INCLUDED__
#include <stdio.h>
#include <iostream>
#endif

#ifndef __OPENCV_H_INCLUDED__   
#define __OPENCV_H_INCLUDED__   
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#endif 


#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

using namespace cv;


// Object colors
// Pink
const Scalar hsv_min_pink_tennis_ball = Scalar(150,0,125);
const Scalar hsv_max_pink_tennis_ball = Scalar(200,125,255);

// Orange
const Scalar hsv_min_orange_hockey_puck = Scalar(2,97,90);
const Scalar hsv_max_orange_hockey_puck = Scalar(8,255,255);

// Cached white object
const Scalar hsv_min_white_cached = Scalar(0,0,166);
const Scalar hsv_max_white_cached = Scalar(150,77,255);

// Cached gray hook
const Scalar hsv_min_gray_cached = Scalar(80,22,102);
const Scalar hsv_max_gray_cached = Scalar(120,74,203);

void identifyColor( Mat& frame ) {

    Mat smoothFrame;
    Mat hsvFrame; 
    Mat threshold;
    
    GaussianBlur(frame, smoothFrame, Size(5,5),0,0);    // Blur
    cvtColor(smoothFrame,hsvFrame,CV_BGR2HSV);          // Convert to HSV

    imshow("Blurred",smoothFrame); 

    // Cached object
    Mat cached_white;
    Mat cached_gray;
    Mat cached_object_frame;
    inRange(hsvFrame,hsv_min_white_cached,hsv_max_white_cached,cached_white);
    inRange(hsvFrame,hsv_min_gray_cached,hsv_max_gray_cached,cached_gray);
    vector<Mat> cached_vec;
    cached_vec.push_back(cached_white);
    cached_vec.push_back(cached_white);
    cached_vec.push_back(cached_gray);
    merge( cached_vec, cached_object_frame );
    imshow("Cached object",cached_object_frame);

    // Pink tennis ball
    Mat tennis_ball_frame;
    inRange(hsvFrame,hsv_min_pink_tennis_ball,hsv_max_pink_tennis_ball,tennis_ball_frame);

   // Blob detection
        SimpleBlobDetector::Params params;
        params.minThreshold = 40;
        params.maxThreshold = 60;
        params.thresholdStep = 5;

        params.minArea = 100; 
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.01;

        params.maxArea = 8000;
        params.maxConvexity = 10;

        params.filterByColor = false;
        params.filterByCircularity = false;

        SimpleBlobDetector blobDetector( params );
        blobDetector.create("SimpleBlob");
        vector<KeyPoint> keyPoints;
        blobDetector.detect(tennis_ball_frame, keyPoints);
        drawKeypoints( tennis_ball_frame, keyPoints, tennis_ball_frame, CV_RGB(0,255,0), DrawMatchesFlags::DEFAULT);

        imshow( "Tennis Ball", tennis_ball_frame );



    // Pink tennis ball
    Mat hockey_puck_frame;
    inRange(hsvFrame,hsv_min_orange_hockey_puck,hsv_max_orange_hockey_puck,hockey_puck_frame);
    imshow("Hockey puck",hockey_puck_frame);

}









