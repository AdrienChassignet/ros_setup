/*
  Copyright 2016 Lucas Walter
*/
#ifndef RQT_EXAMPLE_CPP_MY_PLUGIN_H
#define RQT_EXAMPLE_CPP_MY_PLUGIN_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <sstream>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_example_cpp/ui_my_plugin.h>
#include <QWidget>
//#include <QtWidgets>
#include <QSpinBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QRadioButton>
#include <QGroupBox>


namespace rqt_example_cpp
{

class MyPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  void msgReceivedCallback(const std_msgs::String::ConstPtr& msg);
  void updateState(const std_msgs::String::ConstPtr& msg);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

Q_SIGNALS:
  void receivedString(const QString msg);
  void newState(const QString msg);

private Q_SLOTS:
  void sendCallerToChatter(void);
  void updateLabel(const QString msg);
  void updateRobotState(const QString state);


private:
  Ui::MyPluginWidget ui_;
  QWidget* window_;
  QHBoxLayout *layout_test_; 
  QVBoxLayout *layout_global_;
  QGroupBox *state_group_;
  QSpinBox *box_;
  QLabel *label_;
  QRadioButton *radio_walking_;
  QRadioButton *radio_swimming_;
  QRadioButton *radio_idle_;

  ros::Publisher talker_;
  ros::Subscriber msg_receiver_;
  ros::Subscriber state_updater_;
  ros::NodeHandle nh_;
};
}  // namespace rqt_example_cpp
#endif  // RQT_EXAMPLE_CPP_MY_PLUGIN_H
