#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>


class Core
{
public:
  Core();

private:
  void joyHandlerCallback(const geometry_msgs::Twist::ConstPtr& twist);
  void joyHandler2Callback(const std_msgs::Bool::ConstPtr& isPressed);

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joy_enabled_sub_;
  ros::Publisher vel_pub_;

  bool isEnabled;
};


Core::Core()
{
  isEnabled = false;

  joy_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy_command", 10, &Core::joyHandlerCallback, this);
  joy_enabled_sub_ = nh_.subscribe<std_msgs::Bool>("is_joystick_enabled", 10, &Core::joyHandler2Callback, this);
  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

  while(true){
    ros::spinOnce();
  }
}

void Core::joyHandlerCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  if(isEnabled)
    vel_pub_.publish(twist);
}

void Core::joyHandler2Callback(const std_msgs::Bool::ConstPtr& isPressed)
{
  isEnabled = isPressed->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "core");
  Core Core;
}
