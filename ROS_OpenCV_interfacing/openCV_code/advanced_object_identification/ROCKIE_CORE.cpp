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

#include "displayEdges.hpp"
#include "displayObjects.hpp"
#include "displayCircles.hpp"
#include "displayEllipses.hpp"

#define ROWS 480    // Image size for logitech camera
#define COLS 640 

using namespace cv;

int main(int argc, char** argv) {

    // Variables
    Mat frame;

    // Initialize
    VideoCapture cam(0);
    if(!cam.isOpened()) {  
        printf("ERROR: could not establish camera connection!");
        return -1; 
    }

    namedWindow("Display",1);
    for(;;)
    {
        cam >> frame;               // Capture frame
        //imshow("Display", frame);   // Display raw frame
        displayEdges( frame );      // Display edge detection
        displayCircles( frame );    // Display circles detected
        displayEllipses( frame );    // Display circles detected
        displayObjects( frame );    // Display identified objects 

        if(waitKey(30) >= 0) break;
    }
    return 0;
}





