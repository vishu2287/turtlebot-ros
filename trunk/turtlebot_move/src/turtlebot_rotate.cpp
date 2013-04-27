#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <math.h>
using namespace std;

class TurtlebotMove
{
public:
  typedef std_msgs::Float32MultiArray move_msg_type;
  TurtlebotMove ():
		length_threshold (0.02)
		, linear_speed (0.03)
		, orientation_initialized (false)
  {
    boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb(boost::bind(&TurtlebotMove::odom_callback, this, _1));
    boost::function<void (const move_msg_type::ConstPtr&)> cb1(boost::bind(&TurtlebotMove::turtlebot_move_callback, this, _1));
    odom_combined = nh.subscribe ("/odom_combined", 1000, cb);
    move_instruction = nh.subscribe ("/turtlebot_move_commands", 1000, cb1);
    teleop = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 5);
  }
  void odom_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void turtlebot_move_callback(const TurtlebotMove::move_msg_type::ConstPtr& msg);
  void move();
  void get_current_orientation (Orientation& result);
  void move_straight ();
  void turn ();
  void wait_for_orientation_init ();
  double l2_distance (float x1, float y1, float x2, float y2)
private:
  int sides;
  float length, length_threshold, linear_speed; // metres
  float angle; // degrees
  ros::NodeHandle nh;
  ros::Subscriber odom_combined, move_instruction;
  ros::Publisher teleop;
  bool orientation_initialized;
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
  std::mutex orientation_mutex;
}

void TurtlebotMove::odom_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  orientation_mutex.lock();
  orientation.x = msg->pose.pose.position.x;
  orientation.y = msg->pose.pose.position.y;
  orientation.yaw = yaw;
  if(!orientation_initialized)
    orientation_initialized = true;
  orientation_mutex.unlock();
}

void TurtlebotMove::turtlebot_move_callback (const TurtlebotMove::move_msg_type::ConstPtr& msg)
{
  length = msg->data[0];
  angle = msg->data[1];
  sides = int(msg->data[2]);
  std::cout << "Instruction for move received\nlength: " << length << "\nangle: " << angle << "\nsides: " << sides << endl;
  move();
}

void TurtlebotMove::move ()
{
  int left = sides;
  while(ros::ok() && left>0)
  {
    move_straight();
    turn();
    left--;
  }
}

void wait_for_orientation_init ()
{
  while(1)
  {
    orientation_mutex.lock();
    if(!orientation_initialized)
    {
      orientation_mutex.unlock();
      std::cout << "Waiting for /odom_combined" << endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
      break;
    }
  }
  orientation_mutex.unlock();
}

void TurtlebotMove::get_current_orientation (Orientation& result)
{
//   wait_for_orientation_init ();
  orientation_mutex.lock();
  result = orientation;
  orientation_mutex.unlock();
}

double TurtlebotMove::l2_distance (float x1, float y1, float x2, float y2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void TurtlebotMove::move_straight ()
{
//   wait_for_orientation_init ();
  orientation_mutex.lock();
  float x = orientation.x;
  float y = orientation.y;
  float curx = x, cury = y;
  geometry_msgs::Twist msg;
  msg.linear.x = linear_speed;
  while(ros::ok() && abs(l2_distance(x, y, curx, cury)-length) > length_threshold)
  {
    teleop.publish (msg);
    orientation_mutex.unlock ();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    orientation_mutex.lock ();
    curx = orientation.x;
    cury = orientation.y;
  }
  orientation_mutex.unlock();
}

void TurtlebotMove::turn ()
{
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_move");
  ros::spin();
  return 0;
}