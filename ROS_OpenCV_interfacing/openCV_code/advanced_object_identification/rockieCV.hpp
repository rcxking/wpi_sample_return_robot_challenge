#ifndef rockieCV_H
#define rockieCV_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp" 

using namespace cv;


// 	Useful functions for rockie applications using OpenCV   



//-----------------------
// Capture camera image
void captureCameraImage(Mat& frame) {
		
	VideoCapture cap(0);
    cap >> frame; 

}

 
#endif
