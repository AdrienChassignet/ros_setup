#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <rqt_turtle_teleop/MaxSpeed.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void scaleCallback(const rqt_turtle_teleop::MaxSpeed::ConstPtr& scale);

  ros::NodeHandle nh_;

  int linear_, angular_, lin_vel_, ang_vel_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher scale_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber scale_sub_;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
  // ,lin_vel_(5),
  // ang_vel_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  // scale_pub_ = nh_.advertise<rqt_turtle_teleop::MaxSpeed>("changeScaleSpeed", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
  scale_sub_ = nh_.subscribe<rqt_turtle_teleop::MaxSpeed>("scaleSpeed", 10, &TeleopTurtle::scaleCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);

  // rqt_turtle_teleop::MaxSpeed scale;
  // scale.linear_max_speed = joy->axes[lin_vel_];
  // scale.angular_max_speed = joy->axes[ang_vel_];
  // scale_pub_.publish(scale);
}

void TeleopTurtle::scaleCallback(const rqt_turtle_teleop::MaxSpeed::ConstPtr& scale)
{
  l_scale_ = scale->linear_max_speed;
  a_scale_ = scale->angular_max_speed;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}