#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>


class JoyHandler
{
public:
  JoyHandler();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  bool isEnabled;

  std_msgs::Bool isJoystickEnabled;
  
  ros::Publisher vel_pub_;
  ros::Publisher isJoystickEnabled_pub;

  ros::Subscriber joy_sub_;

};


JoyHandler::JoyHandler(): linear_(1), angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("joy_command", 1);
  isJoystickEnabled_pub = nh_.advertise<std_msgs::Bool>("is_joystick_enabled",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyHandler::joyCallback, this);
}

void JoyHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];  
  vel_pub_.publish(twist);
  
  if(joy->buttons[0]){
    isJoystickEnabled.data = true;
    isJoystickEnabled_pub.publish(isJoystickEnabled);
  }
  else if(joy->buttons[1]){
    isJoystickEnabled.data = false;
    isJoystickEnabled_pub.publish(isJoystickEnabled);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "JoyHandler");
  JoyHandler JoyHandler;

  ros::spin();
}
