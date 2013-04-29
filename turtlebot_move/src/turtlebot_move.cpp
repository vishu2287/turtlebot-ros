#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <algorithm>
#include <turtlebot_move/Moves.h>
using namespace std;

enum MOVE_TYPE {LINEAR, ANGULAR};
struct Move
{
  MOVE_TYPE type;
  float value;
};
struct Moves
{
  vector<Move> moves;
};

class TurtlebotMove
{
public:
  typedef typename turtlebot_move::Moves move_msg_type;
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
		length_threshold (0.005f)
		, max_linear_speed (0.2f)
		, min_linear_speed (-0.2f)
		, min_positive_linear_speed (0.02f)
		, max_negative_linear_speed (-0.02f)
		, max_angular_speed (0.7f)
		, min_angular_speed (-0.7f)
		, min_positive_angular_speed (0.15f)
		, max_negative_angular_speed (-0.15f)
		, ros_rate (20.0)
		, angular_scale (3.60/360) // degrees to odom scale
		, angular_threshold (0.02f)
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
  float length, length_threshold, max_linear_speed, min_linear_speed, min_positive_linear_speed, max_negative_linear_speed; // metres
  float angle/*degrees*/, angular_speed, angular_scale, angular_threshold, max_angular_speed, min_angular_speed, min_positive_angular_speed, max_negative_angular_speed;
  const float PI;
  ros::NodeHandle nh;
  ros::Subscriber move_instruction;
  ros::Publisher teleop;
  float ros_rate;
  Moves moves;
};



void TurtlebotMove::turtlebot_move_callback (const TurtlebotMove::move_msg_type::ConstPtr& msg)
{
  moves.moves.clear();
  for(int i=0;i<msg->moves.size() && nh.ok();i++)
  {
    Move move;
    if((*msg).moves[i].type.compare("linear")==0 || (*msg).moves[i].type.compare("Linear")==0)
    {
      move.type = LINEAR;
    }
    else if((*msg).moves[i].type.compare("angular")==0 || (*msg).moves[i].type.compare("Angular")==0)
    {
      move.type = ANGULAR;
    }
    else
    {
      continue;
    }
    move.value = msg->moves[i].value;
    moves.moves.push_back(move);
  }
  move();
}

void TurtlebotMove::move ()
{
  for(int i=0; i<moves.moves.size() && nh.ok(); i++)
  {
    if(moves.moves[i].type==LINEAR)
    {
      cout << "Moving linearly for distance " << moves.moves[i].value << "m" << endl;
      move_straight(moves.moves[i].value);
    }
    else if(moves.moves[i].type==ANGULAR)
    {
      cout << "Rotating by " << moves.moves[i].value << " degrees" << endl;
      turn(moves.moves[i].value);
    }
  }
  cout << "Move complete" << endl;
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
  bool reverse = false;
  if(len<0)
    reverse = true;
  len = abs(len);
  
  while (nh.ok() && abs(len-d)>length_threshold){
    geometry_msgs::Twist msg;
    if(len>d)
    {
      msg.linear.x = max(min((len-d), max_linear_speed), min_positive_linear_speed);
    }
    else
    {
      msg.linear.x = min(max((len-d), min_linear_speed), max_negative_linear_speed);
    }
    if(reverse)
      msg.linear.x = -msg.linear.x;
    
    printf("Sending message to move with linear speed: %f\n", msg.linear.x);
    
    teleop.publish (msg);

    rate.sleep();
    
    while(!get_current_orientation(curo))
    {
      printf("Waiting for transform\n");
    }
    d = l2_distance(o.x, o.y, curo.x, curo.y);
    printf("Done %f\n", d);
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
  bool reverse = false;
  if(ang<0)
  {
    reverse = true;
  }
  ang = abs(ang);
  float to_rotate = angular_scale*ang;
  printf("Beginning to rotate by %f\n", reverse?-to_rotate:to_rotate);
  
  while (nh.ok() && abs(to_rotate-abs(done))>angular_threshold){
    geometry_msgs::Twist msg;
    float factor = 2.0f;
    if(to_rotate>abs(done))
    {
      msg.angular.z = max(min(factor*(to_rotate-abs(done)), max_angular_speed), min_positive_angular_speed);
    }
    else
    {
      msg.angular.z = min(max(factor*(to_rotate-abs(done)), min_angular_speed), max_negative_angular_speed);
    }
    if(reverse)
      msg.angular.z = -msg.angular.z;
    
    printf("Sending message to move with angular speed: %f\n", msg.angular.z);
    
    teleop.publish (msg);

    rate.sleep();
    
    while(!get_current_orientation(curo))
    {
      printf("Waiting for transform\n");
    }
    
    float temp = curo.yaw-prevo.yaw;
    if(temp<-3)
    {
      temp+=2*PI;
    }
    else if(temp>3)
    {
      temp-=2*PI;
    }
    done += temp;
    
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