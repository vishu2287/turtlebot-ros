#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "turtlebot_tf_listener");

  ros::NodeHandle node;

  ros::Publisher turtlebot_vel = 
    node.advertise<geometry_msgs::PoseWithCovarianceStamped>("turtlebot_test", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/world", "/turtlebot",
                              now, ros::Duration(3.0));
      listener.lookupTransform("/world", "/turtlebot",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Caught in Exception: %s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.position.x = transform.getOrigin().x();
    msg.pose.pose.position.y = transform.getOrigin().y();
    turtlebot_vel.publish(msg);

    rate.sleep();
  }
  return 0;
};
