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


using namespace cv;

// Global Variables
const int slider_max = 255;
int h_min_slider;
int h_max_slider;
int v_min_slider;
int v_max_slider;
int s_min_slider;
int s_max_slider;
double alpha;
double beta;

double h_min;
double h_max;
double s_min;
double s_max;
double v_min;
double v_max;


// Trackbar callback functions
void H_min( int, void* ) {
    h_min = (double) h_min_slider;
}
void H_max( int, void* ) {
    h_max = (double) h_max_slider;
}
void S_min( int, void* ) {
    s_min = (double) s_min_slider;
}
void S_max( int, void* ) {
    s_max = (double) s_max_slider;
}
void V_min( int, void* ) {
    v_min = (double) v_min_slider;
}
void V_max( int, void* ) {
    v_max = (double) v_max_slider;
}

////////////////////////////////////
// colorFinder
void colorFinder( Mat& frame ) {

    // Create trackbars
    createTrackbar( "Hmin", "Color Finder", &h_min_slider, slider_max, H_min );
    createTrackbar( "Hmax", "Color Finder", &h_max_slider, slider_max, H_max );
    createTrackbar( "Smin", "Color Finder", &s_min_slider, slider_max, S_min );
    createTrackbar( "Smax", "Color Finder", &s_max_slider, slider_max, S_max );
    createTrackbar( "Vmin", "Color Finder", &v_min_slider, slider_max, V_min );
    createTrackbar( "Vmax", "Color Finder", &v_max_slider, slider_max, V_max );

    
    Mat hsvFrame;
    Mat threshold;

    cvtColor(frame,hsvFrame,CV_BGR2HSV);
    inRange(hsvFrame,Scalar(h_min,s_min,v_min),Scalar(h_max,s_max,v_max),threshold);
    imshow("Color Finder",threshold); 
}









