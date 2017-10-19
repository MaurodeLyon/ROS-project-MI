#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class Core
{
public:
  Core();

private:
  void joyHandlerCallback(const geometry_msgs::Twist::ConstPtr& twist);

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher vel_pub_;
};


Core::Core()
{
  joy_sub_ = nh_.subscribe<geometry_msgs::Twist>("joy_command", 10, &Core::joyHandlerCallback, this);
  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

  while(true){
    ros::spinOnce();
  }
}

void Core::joyHandlerCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "core");
  Core Core;
}
