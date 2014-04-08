#ifndef __OPENCV_H_INCLUDED__   
#define __OPENCV_H_INCLUDED__   
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#endif 

#include "identifyObject.hpp"

using namespace cv;

void displayObjects( Mat frame ) {

   
    // Load object images
    Mat drill1 = imread( "drill1.jpg", CV_LOAD_IMAGE_GRAYSCALE ); 
    Mat drill2 = imread( "drill2.jpg", CV_LOAD_IMAGE_GRAYSCALE ); 
    Mat mug = imread( "mug.jpg", CV_LOAD_IMAGE_GRAYSCALE ); 
    Mat puck = imread( "puck.jpg", CV_LOAD_IMAGE_GRAYSCALE ); 
    Mat tire = imread( "tire.jpg", CV_LOAD_IMAGE_GRAYSCALE ); 
    Mat oBall = imread( "orangeBall.jpg", CV_LOAD_IMAGE_GRAYSCALE ); 

    // Identify objects
    identifyObject(frame, puck, "puck" );
    //identifyObject(frame, mug, "mug" );
    //identifyObject(frame, oBall, "Ball" );
    //identifyObject(frame, drill1, "drill1" );
    //identifyObject(frame, drill2, "drill2" );
    //identifyObject(frame, tire, "tire" );

    // Display 
    imshow("Objects",frame); 

}









