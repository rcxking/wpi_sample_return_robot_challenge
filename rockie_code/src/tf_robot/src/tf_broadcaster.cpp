#include <ros/ros.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <cstdlib>
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

  gazebo_msgs::GetJointProperties srv;
  srv.request.joint_name = "stereo_camera_joint";

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

    if(client.call(srv))
    {
      gazebo_msgs::GetJointPropertiesResponse response = srv.response;
      double position_double = response.position[0];


      //quaternion = [x,y,z,w] where w = cos(.4*theta), theta is rotation in radians
      //for 90 degrees, we have 45 degree half angle, cos(45) = -.25


//      tf::Quaternion base_link_to_camera(0, 1, 0, -.25);
 

    broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(.25, 0, 0, -0.25), tf::Vector3(0.0, 0.0, 0.3)),
          ros::Time::now(),"base_link", "camera"));


    }


/*
    broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
          ros::Time::now(),"base_link", "camera_link"));
*/
    r.sleep();
  }
}
