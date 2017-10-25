#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class Core
{
public:
  Core();

private:
  void joyHandlerCallback(const geometry_msgs::Twist::ConstPtr& twist);
  void joyHandler2Callback(const std_msgs::Bool::ConstPtr& isPressed);
  void receivingDirections(const std_msgs::String::ConstPtr& msg);
  void straight();
  void turningOnPointRight();
  void goingLeft();
  void goingRight();
  void stop();

  ros::NodeHandle nh_;
  ros::Subscriber joy_direction_sub_;
  ros::Subscriber joy_enabled_sub_;
  ros::Subscriber camera_direction_sub_;
  ros::Publisher rosaria_pub_;

  bool isEnabled;
};


Core::Core()
{
  isEnabled = false;

  joy_direction_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy_command", 10, &Core::joyHandlerCallback, this);
  joy_enabled_sub_ = nh_.subscribe<std_msgs::Bool>("is_joystick_enabled", 10, &Core::joyHandler2Callback, this);
  camera_direction_sub_ = nh_.subscribe("/camera_directions", 10, &Core::receivingDirections,this);
   
  rosaria_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);

  while(true){
    ros::spinOnce();
  }
}

void Core::joyHandlerCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  if(isEnabled)
    rosaria_pub_.publish(twist);
}

void Core::joyHandler2Callback(const std_msgs::Bool::ConstPtr& isPressed)
{
  isEnabled = isPressed->data;
}

void Core::receivingDirections(const std_msgs::String::ConstPtr& msg)
{
  if(!isEnabled){
	if(msg->data == "CENTER")
	{
		straight();
	}
	else if(msg->data == "LEFT")
	{
		goingLeft();
	}
	else if(msg->data == "RIGHT")
	{
		goingRight();
	}
	else if(msg->data == "NONE")
	{
		turningOnPointRight();
	}
	else if(msg->data == "FOUND")
	{
		stop();
	}
  }
}

void Core::straight()
{
   	geometry_msgs::Twist msg;
    msg.linear.x = .25;
    rosaria_pub_.publish(msg);
}

void Core::stop()
{
   	geometry_msgs::Twist msg;
    msg.linear.x = .0;
    rosaria_pub_.publish(msg);
}

void Core::turningOnPointRight()
{
    geometry_msgs::Twist msg;
    msg.angular.z = -.25;
    rosaria_pub_.publish(msg);
}

void Core::goingLeft()
{
    geometry_msgs::Twist msg;
    msg.angular.z = .2;
    msg.linear.x = .1;
    rosaria_pub_.publish(msg);
}

void Core::goingRight()
{
    geometry_msgs::Twist msg;
    msg.angular.z = -.2;
	msg.linear.x = .1;
    rosaria_pub_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "core");
  Core Core;
}
