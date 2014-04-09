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

void identifyObject( Mat& frame, Mat& object, const string& objectName ) {

    //Detect the keypoints using SURF Detector
    int minHessian = 500;
    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> kp_object;
    detector.detect( object, kp_object );

    //Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat des_object;
    extractor.compute( object, kp_object, des_object );
    FlannBasedMatcher matcher;


    //Get the corners from the object
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( object.cols, 0 );
    obj_corners[2] = cvPoint( object.cols, object.rows );
    obj_corners[3] = cvPoint( 0, object.rows );


    // Match descriptors to frame
    Mat des_image, img_matches;
    std::vector<KeyPoint> kp_image;
    std::vector<vector<DMatch > > matches;
    std::vector<DMatch > good_matches;
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    std::vector<Point2f> scene_corners(4);
    Mat H;
    Mat image;

    cvtColor(frame, image, CV_RGB2GRAY);

    detector.detect( image, kp_image );
    extractor.compute( image, kp_image, des_image );

    matcher.knnMatch(des_object, des_image, matches, 2);

    for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
    {
        if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
        {
            good_matches.push_back(matches[i][0]);
        }
    }

    //Draw only "good" matches
    drawMatches( object, kp_object, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    if (good_matches.size() >= 4)
    {
        for( int i = 0; i < good_matches.size(); i++ )
        {
            //Get the keypoints from the good matches
            obj.push_back( kp_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
        }

        H = findHomography( obj, scene, CV_RANSAC );

        perspectiveTransform( obj_corners, scene_corners, H);

        //Draw lines between the corners (the mapped object in the scene image )
        line( frame, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
        line( frame, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 4 );
        line( frame, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 4 );
        line( frame, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );
    }

    //Show detected matches
    Point2f textPoint = cvPoint( (scene_corners[0].x+scene_corners[1].x+scene_corners[2].x+scene_corners[3].x )/4.0 , (scene_corners[0].y+scene_corners[1].y+scene_corners[2].y+scene_corners[3].y )/4.0 );
    putText( frame, objectName, textPoint, FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0,250,150), 1, CV_AA );

}









