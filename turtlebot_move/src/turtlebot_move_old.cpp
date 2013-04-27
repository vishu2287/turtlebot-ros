#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <math.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
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
		, orientation_initialized (false)
		, orientation_updated (false)
  {
    boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb(boost::bind(&TurtlebotMove::odom_callback, this, _1));
    boost::function<void (const move_msg_type::ConstPtr&)> cb1(boost::bind(&TurtlebotMove::turtlebot_move_callback, this, _1));
    odom_combined = nh.subscribe ("/robot_pose_ekf/odom_combined", 1, cb);
    move_instruction = nh.subscribe ("/turtlebot_move_commands", 1, cb1);
    teleop = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  }
  void odom_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void turtlebot_move_callback(const TurtlebotMove::move_msg_type::ConstPtr& msg);
  void move();
  void get_current_orientation (Orientation& result);
  void move_straight ();
  void turn ();
  void wait_for_orientation_init ();
  float l2_distance (float x1, float y1, float x2, float y2);
  void run ();
private:
  int sides;
  float length, length_threshold, linear_speed; // metres
  float angle; // degrees
  ros::NodeHandle nh;
  ros::Subscriber odom_combined, move_instruction;
  ros::Publisher teleop;
  bool orientation_initialized;
  
//   boost::mutex orientation_mutex;
//   boost::condition_variable cvar;
//   bool orientation_updated;
};

void TurtlebotMove::odom_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//   orientation_mutex.lock();
  cout << "Odom Callback Trying to lock" << endl;
//   boost::lock_guard<boost::mutex> lock (orientation_mutex);
  orientation_updated = true;
  cout << "Odom Callback" << endl;
  orientation.x = msg->pose.pose.position.x;
  orientation.y = msg->pose.pose.position.y;
  orientation.yaw = yaw;
  if(!orientation_initialized)
  {
    orientation_initialized = true;
  }
//   cvar.notify_one();
}

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

void TurtlebotMove::wait_for_orientation_init ()
{
  while(1)
  {
    orientation_mutex.lock();
    if(!orientation_initialized)
    {
      orientation_mutex.unlock();
      std::cout << "Waiting for /odom_combined" << endl;
      usleep(500);
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

float TurtlebotMove::l2_distance (float x1, float y1, float x2, float y2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void TurtlebotMove::move_straight ()
{
//   wait_for_orientation_init ();
//   orientation_mutex.lock();
//   boost::unique_lock<boost::mutex> lock(orientation_mutex);
//   while (!orientation_updated)
//   {
//     cvar.wait (lock);
//   }
  float x = orientation.x;
  float y = orientation.y;
  float curx = x, cury = y;
  geometry_msgs::Twist msg;
  msg.linear.x = linear_speed;
  float d = l2_distance(x, y, curx, cury);
  while (ros::ok() && d<length)
  {
//     cout << "Current distance: " << d << endl;
    teleop.publish (msg);
//     while (!orientation_updated)
//     {
//       cout << "Sleeping!" << endl;
//       cvar.wait (lock);
//     }
    curx = orientation.x;
    cury = orientation.y;
    d = l2_distance (x, y, curx, cury);
    orientation_updated = false;
  }
//   orientation_mutex.unlock();
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