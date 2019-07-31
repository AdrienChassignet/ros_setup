#ifndef RQT_TURTLE_TELEOP_MY_PLUGIN_H
#define RQT_TURTLE_TELEOP_MY_PLUGIN_H

#include "ros/ros.h"
#include <rqt_gui_cpp/plugin.h>
#include <rqt_turtle_teleop/ui_my_plugin.h>
#include <rqt_turtle_teleop/MaxSpeed.h>
#include <QWidget>
#include <QtWidgets>

namespace rqt_turtle_teleop {

#define LIN_MAX 10
#define LIN_MIN 0
#define LIN_INI 2
#define ANG_MAX 10
#define ANG_MIN 0
#define ANG_INI 2

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  // void changeMaxCallback(const rqt_turtle_teleop::MaxSpeed::ConstPtr& val);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private Q_SLOTS:
  void updateMaxSpeed(int val);

private:
  Ui::MyPluginWidget ui_;
  QWidget* window_;
  QSpinBox* box1_;
  QSpinBox* box2_;
  QSlider* slider1_;
  QSlider* slider2_;

  int linear_max_speed_;
  int angular_max_speed_;

  ros::NodeHandle nh_;
  ros::Publisher max_speed_pub_;
  // ros::Subscriber change_max_sub_;
};
} // namespace
#endif // my_namespace__my_plugin_H