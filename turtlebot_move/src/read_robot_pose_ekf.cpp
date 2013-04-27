#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
using namespace std;

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
//   btQuaternion q;
//   double roll, pitch, yaw;
//   tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
//   btMatrix3x3(q).getRPY(roll, pitch, yaw);
//   ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
  
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  cout << "RPY = (" << roll << ", " << pitch << ", " << yaw << ")" << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_robot_pose_ekf");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom_combined", 1000, callback);
  ros::spin();

  return 0;
}