#include <ros/ros.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <cstdlib>
#include <tf/transform_broadcaster.h>

//void poseCallback(const PoseStamped)

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/GetJointProperties")

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  //ros::Subscriber sub = n.subscribe("stereo_odometer/pose", 10, &poseCallback)

  while(n.ok()){



    broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
          ros::Time::now(),"base_link", "camera_link"));

    r.sleep();
  }
}
