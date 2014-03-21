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

  //ros::Publisher echo_test = n.advertise<std_msgs::String>("will_test", 1000);

  while(n.ok()){

    if(client.call(srv))
    {
      gazebo_msgs::GetJointPropertiesResponse response = srv.response;
      ostringstream ss;
      double position_double = response.position[0];
      ss << position_double;
      string s(ss.str());
      ROS_INFO_STREAM(s);
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
