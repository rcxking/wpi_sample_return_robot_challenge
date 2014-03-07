#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image Processed";

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = \
                       255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }


    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    cv::destroyWindow(WINDOW);
    pub = it.advertise("camera/image_processed", 1);
    ros::spin();
    ROS_INFO("testcamera::main.cpp::No error.");
}
