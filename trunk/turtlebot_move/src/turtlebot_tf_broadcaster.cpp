#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


void odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/turtlebot"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "turtlebot_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/odom_combined", 100, &odom_callback);

  ros::spin();
  return 0;
};