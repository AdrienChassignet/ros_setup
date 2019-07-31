#include <string>
#include <vector>
#include <typeinfo>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>

#define MAX_POS 2550
#define ZERO_POS 2048
#define MIN_POS 1550

class Joy2Dynamixel
{
public:
  Joy2Dynamixel();
  void updateJointTraj(void);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int fb_left_joy_, lr_left_joy_, square_button_, circ_button_, cross_button_;
  int dyn_pos_;
  trajectory_msgs::JointTrajectory new_point_;

  ros::Subscriber joy_sub_;
  ros::Publisher joint_pub_;
  // ros::ServiceClient dyn_cmd_client_;
};


Joy2Dynamixel::Joy2Dynamixel():
  fb_left_joy_(1),
  lr_left_joy_(0),
  square_button_(3),
  circ_button_(1),
  cross_button_(0),
  dyn_pos_(ZERO_POS)
{
  nh_.param("dynamixel_position", dyn_pos_, dyn_pos_);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy2Dynamixel::joyCallback, this);
  joint_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("dynamixel_workbench/joint_trajectory", 1);
  // dyn_cmd_client_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_command");
  
  new_point_.points.resize(1);
  new_point_.points[0].positions.resize(1);
  new_point_.points[0].velocities.resize(1);
  new_point_.joint_names.push_back("servo");
  new_point_.points[0].positions[0] = (dyn_pos_-2048)*3.14/2048;
  new_point_.points[0].velocities[0] = 0.1;
  new_point_.points[0].time_from_start = ros::Duration(0.2);

  ROS_INFO("Init");
}

void Joy2Dynamixel::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  int new_pos = dyn_pos_;
  new_point_.points[0].time_from_start = ros::Duration(0.5);

  if(joy->buttons[square_button_])
    new_pos = MIN_POS;
  else if(joy->buttons[cross_button_])
    new_pos = ZERO_POS;
  else if(joy->buttons[circ_button_])
    new_pos = MAX_POS;
  else
  {
    new_pos = ZERO_POS - (MAX_POS-ZERO_POS)*(joy->axes[lr_left_joy_]);
    if(new_pos > MAX_POS)
        new_pos = MAX_POS;
    if(new_pos < MIN_POS)
        new_pos = MIN_POS;
  }

  if(new_pos != dyn_pos_)
  {
    dyn_pos_ = new_pos;
    ROS_INFO("in joy callback, position: %d", dyn_pos_);

    if(ros::ok() && joint_pub_)
    {
      // trajectory_msgs::JointTrajectory new_point;
      // new_point_.points.resize(1);
      // new_point_.points[0].positions.resize(1);
      // new_point_.joint_names.push_back("servo");
      new_point_.points[0].time_from_start = ros::Duration(0.2);
      new_point_.points[0].positions[0] = (dyn_pos_-2048)*3.14/2048;
      // new_point_.points[0].time_from_start = ros::Duration(0.2);
      // joint_pub_.publish(new_point);
    }
    // dynamixel_workbench_msgs::DynamixelCommand cmd_srv;
    // cmd_srv.request.command = "";
    // cmd_srv.request.id = 1;
    // cmd_srv.request.addr_name = "Goal_Position";
    // cmd_srv.request.value = dyn_pos_;
    // ROS_INFO("%s, %d, %s, %d", cmd_srv.request.command.c_str(), cmd_srv.request.id, cmd_srv.request.addr_name.c_str(), cmd_srv.request.value);
  
    // if(dyn_cmd_client_.call(cmd_srv))
    //     ROS_INFO("Message sent, position : %d", dyn_pos_);
    // else
    //     ROS_INFO_STREAM("Failed to call the dynamixel server" << cmd_srv.response);
  }
}

void Joy2Dynamixel::updateJointTraj()
{
  joint_pub_.publish(new_point_);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_2_dynamixel");
  Joy2Dynamixel joy_2_dynamixel;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    joy_2_dynamixel.updateJointTraj();
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ros::spin();
}
