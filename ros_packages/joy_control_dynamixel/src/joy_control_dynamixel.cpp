#include <string>
#include <vector>
#include <typeinfo>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace joy2dynamixel
{
  
  #define ZERO_POS 2048
  #define MIN_POS(amp) (ZERO_POS-amp/2)
  #define MAX_POS(amp) (ZERO_POS+amp/2)

  class Joy2Dynamixel
  {
  public:
    Joy2Dynamixel();
    void updateJointTraj(void);

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void ampCallback(const std_msgs::Int16::ConstPtr& amp);

    ros::NodeHandle nh_;

    int fb_left_joy_, lr_left_joy_, square_button_, circ_button_, cross_button_;
    int amplitude_, dyn_pos_;
    trajectory_msgs::JointTrajectory new_point_;
    
    ros::Subscriber joy_sub_;
    ros::Subscriber ampli_sub_;
    ros::Publisher joint_pub_;
    // ros::ServiceClient dyn_cmd_client_;
  };


  Joy2Dynamixel::Joy2Dynamixel():
    fb_left_joy_(1),
    lr_left_joy_(0),
    square_button_(3),
    circ_button_(1),
    cross_button_(0),
    dyn_pos_(ZERO_POS),
    amplitude_(600)
  {
    nh_.param("dynamixel_position", dyn_pos_, dyn_pos_);
    nh_.param("dynamixel_amplitude", amplitude_, amplitude_);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy2Dynamixel::joyCallback, this);
    ampli_sub_ = nh_.subscribe<std_msgs::Int16>("joy2dynamixel/amplitude", 10, &Joy2Dynamixel::ampCallback, this);
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

    if(joy->buttons[square_button_])
      new_pos = MIN_POS(amplitude_);
    else if(joy->buttons[cross_button_])
      new_pos = ZERO_POS;
    else if(joy->buttons[circ_button_])
      new_pos = MAX_POS(amplitude_);
    else
    {
      new_pos = ZERO_POS - (MAX_POS(amplitude_)-ZERO_POS)*(joy->axes[lr_left_joy_]);
      if(new_pos > MAX_POS(amplitude_))
          new_pos = MAX_POS(amplitude_);
      if(new_pos < MIN_POS(amplitude_))
          new_pos = MIN_POS(amplitude_);
    }

    if(new_pos != dyn_pos_)
    {
      dyn_pos_ = new_pos;
      ROS_INFO("in joy callback, position: %d", dyn_pos_);

      if(ros::ok() && joint_pub_)
      {
        new_point_.points[0].positions[0] = (dyn_pos_-2048)*3.14/2048;
      }
    }
  }
  
  void Joy2Dynamixel::ampCallback(const std_msgs::Int16::ConstPtr& amp)
  {
    amplitude_ = amp->data;
  }

  void Joy2Dynamixel::updateJointTraj()
  {
    joint_pub_.publish(new_point_);
  }
} //namespace joy2dynamixel

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_2_dynamixel");
  joy2dynamixel::Joy2Dynamixel joy_2_dynamixel;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    joy_2_dynamixel.updateJointTraj();
    ros::spinOnce();
    loop_rate.sleep();
  }
    // ros::spin();
}
