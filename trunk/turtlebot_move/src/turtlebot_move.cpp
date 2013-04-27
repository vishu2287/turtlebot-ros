#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <math.h>
#include <tf/transform_listener.h>
using namespace std;

class TurtlebotMove
{
public:
  typedef typename geometry_msgs::Twist move_msg_type;
  struct Orientation
  {
    float x, y, yaw;
    Orientation& operator= (const Orientation& o)
    {
      x = o.x;
      y = o.y;
      yaw = o.yaw;
      return *this;
    }
  } orientation;
  TurtlebotMove ():
		length_threshold (0.02)
		, linear_speed (0.1)
  {
    boost::function<void (const move_msg_type::ConstPtr&)> cb1(boost::bind(&TurtlebotMove::turtlebot_move_callback, this, _1));
    move_instruction = nh.subscribe ("/turtlebot_move_commands", 1, cb1);
    teleop = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 5);
  }
  void turtlebot_move_callback(const TurtlebotMove::move_msg_type::ConstPtr& msg);
  void move();
  void get_current_orientation (Orientation& result);
  void move_straight ();
  void turn ();
  float l2_distance (float x1, float y1, float x2, float y2);
  void run ();
private:
  int sides;
  float length, length_threshold, linear_speed; // metres
  float angle, angular_speed; // degrees
  ros::NodeHandle nh;
  ros::Subscriber move_instruction;
  ros::Publisher teleop;
};

void TurtlebotMove::turtlebot_move_callback (const TurtlebotMove::move_msg_type::ConstPtr& msg)
{
  length = msg->linear.x;
  angle = msg->linear.y;
  sides = msg->linear.z;
  std::cout << "Instruction for move received\nlength: " << length << "\nangle: " << angle << "\nsides: " << sides << endl;
  move();
}

void TurtlebotMove::move ()
{
  for(int i=1; i<=sides && ros::ok(); i++)
  {
    cout << "Moving on side " << i << endl;
    move_straight();
    turn();
  }
}

void TurtlebotMove::get_current_orientation (Orientation& result)
{
}

float TurtlebotMove::l2_distance (float x1, float y1, float x2, float y2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void TurtlebotMove::move_straight ()
{
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  tf::StampedTransform transform;
  std::string source_frame = "/world";
  std::string target_frame = "/turtlebot";
  
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform(source_frame, target_frame,
                              now, ros::Duration(3.0));
    listener.lookupTransform(source_frame, target_frame,  
			      ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  float x = transform.getOrigin().x();
  float y = transform.getOrigin().y();
  float curx = x, cury = y;
  float d = 0;
  
  while (nh.ok() && d<length){
    geometry_msgs::Twist msg;
    msg.linear.x = length-d;
    
    ROS_INFO("Sending message to move: %d\n", msg.linear.x);
    
    teleop.publish (msg);

    rate.sleep();
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(source_frame, target_frame,  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    curx = transform.getOrigin().x();
    cury = transform.getOrigin().y();
    d = l2_distance(x, y, curx, cury);
  }
}

void TurtlebotMove::turn ()
{
  
}

void TurtlebotMove::run ()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "turtlebot_move");
  TurtlebotMove obj;
  obj.run();
  return 0;
}