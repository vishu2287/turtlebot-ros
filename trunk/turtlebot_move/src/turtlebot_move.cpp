#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <algorithm>
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
  };
  TurtlebotMove ():
		length_threshold (0.02f)
		, max_linear_speed (0.1f)
		, max_angular_speed (0.5f)
		, min_angular_speed (-0.5f)
		, ros_rate (20.0)
		, angular_scale (3.52/360) // degrees to odom scale
		, angular_threshold (0.1f)
		, PI (3.141592f)
  {
    boost::function<void (const move_msg_type::ConstPtr&)> cb1(boost::bind(&TurtlebotMove::turtlebot_move_callback, this, _1));
    move_instruction = nh.subscribe ("/turtlebot_move_commands", 1, cb1);
    teleop = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 5);
  }
  void turtlebot_move_callback(const TurtlebotMove::move_msg_type::ConstPtr& msg);
  void move();
  bool get_current_orientation (Orientation&);
  void move_straight (float);
  void turn (float);
  float l2_distance (float x1, float y1, float x2, float y2);
  void run ();
private:
  int sides;
  float length, length_threshold, max_linear_speed; // metres
  float angle/*degrees*/, angular_speed, angular_scale, angular_threshold, max_angular_speed, min_angular_speed;
  const float PI;
  ros::NodeHandle nh;
  ros::Subscriber move_instruction;
  ros::Publisher teleop;
  float ros_rate;
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
    move_straight(length);
    cout << "Turning" << endl;
    turn(angle);
    cout << "Move completed" << endl;
  }
}

bool TurtlebotMove::get_current_orientation (Orientation& o)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string source_frame = "/world";
  std::string target_frame = "/turtlebot";
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform(source_frame, target_frame,
                              now, ros::Duration(0.2));
    listener.lookupTransform(source_frame, target_frame,  
			      ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
//     ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  o.x = transform.getOrigin().x();
  o.y = transform.getOrigin().y();
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  o.yaw = yaw;
//   printf("Yaw: %f\n", o.yaw);
  return true;
}

float TurtlebotMove::l2_distance (float x1, float y1, float x2, float y2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void TurtlebotMove::move_straight (float len)
{
  ros::Rate rate(ros_rate);
  Orientation o, curo;
  
  while(!get_current_orientation(o))
  {
    printf("Waiting for transform\n");
  }
  
  curo = o;
  float d = 0;
  
  while (nh.ok() && abs(len-d)>length_threshold){
    geometry_msgs::Twist msg;
    msg.linear.x = min((len-d), max_linear_speed);
    
    printf("Sending message to move with linear speed: %f\n", msg.linear.x);
    
    teleop.publish (msg);

    rate.sleep();
    
    while(!get_current_orientation(curo))
    {
      printf("Waiting for transform\n");
    }
    d = l2_distance(o.x, o.y, curo.x, curo.y);
  }
}

/**
 * @arg ang : +ve for anti-clockwise
 */
void TurtlebotMove::turn (float ang)
{
  ros::Rate rate(ros_rate);
  Orientation o, curo, prevo;
  
  while(!get_current_orientation(o))
  {
    printf("Waiting for transform\n");
  }
  
  curo = o;
  prevo = o;
  float done = 0.0f;
  float to_rotate = angular_scale*ang;
  printf("Beginning to rotate by %f\n", to_rotate);
  
  while (nh.ok() && abs(to_rotate-done)>angular_threshold){
    geometry_msgs::Twist msg;
    if(to_rotate>0)
    {
      msg.angular.z = min((to_rotate-done), max_angular_speed);
    }
    else
    {
      msg.angular.z = max((to_rotate-done), min_angular_speed);
    }
    printf("Sending message to move with angular speed: %f\n", msg.angular.z);
    
    teleop.publish (msg);

    rate.sleep();
    
    while(!get_current_orientation(curo))
    {
      printf("Waiting for transform\n");
    }
    
    float temp = curo.yaw-prevo.yaw;
    done += temp;
    if(msg.angular.z>0 && temp<-3)
    {
      done+=PI;
    }
    else if(msg.angular.z<0 && temp>3)
    {
      done-=PI;
    }
    
    prevo = curo;
    printf("Done %f\n", done);
  }
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