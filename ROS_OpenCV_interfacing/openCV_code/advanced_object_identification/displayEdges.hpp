
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

void displayEdges( Mat &frame ) {

    // Display edge detection
    Mat frame_edges; 

    cvtColor(frame, frame_edges, CV_BGR2GRAY);
    GaussianBlur(frame_edges, frame_edges, Size(7,7), 1.5, 1.5);
    Canny(frame_edges, frame_edges, 0, 30, 3);
    imshow("Edges", frame_edges);

}
