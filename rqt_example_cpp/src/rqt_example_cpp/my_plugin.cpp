/*
  Copyright 2016 Lucas Walter
*/

#include "rqt_example_cpp/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_example_cpp
{

void MyPlugin::msgReceivedCallback(const std_msgs::String::ConstPtr& msg)
{
  /**
   *  Function called when the subscriber received a message on the /label topic
   */
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  //emit a signal with the message received
  emit MyPlugin::receivedString(msg->data.c_str());
}

void MyPlugin::updateState(const std_msgs::String::ConstPtr& msg)
{
  /**
   *  Callback to update the state of the robot in the gui according to the 
   * published message on the /robot_state topic
   */
  ROS_INFO("Robot state update");
  //emit a signal with the state passed in argument
  emit MyPlugin::newState(msg->data.c_str());
}


MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , window_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ROS_INFO("Starting plugin");

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget

  box_ = new QSpinBox();
  label_ = new QLabel("Wesh");
  radio_walking_ = new QRadioButton("Walking");
  radio_swimming_ = new QRadioButton("Swimming");
  radio_idle_ = new QRadioButton("Idle");
  radio_idle_->setChecked(true);

  layout_test_ = new QHBoxLayout;
  layout_test_->addWidget(box_);
  layout_test_->addWidget(label_);

  state_group_ = new QGroupBox(tr("State of the robot"));
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(radio_walking_);
  vbox->addWidget(radio_swimming_);
  vbox->addWidget(radio_idle_);
  vbox->addStretch(1);
  state_group_->setLayout(vbox);

  layout_global_ = new QVBoxLayout;
  layout_global_->addLayout(layout_test_);
  layout_global_->addWidget(state_group_);

  msg_receiver_ = nh_.subscribe("label", 10, &MyPlugin::msgReceivedCallback, this);
  state_updater_ = nh_.subscribe("robot_state", 10, &MyPlugin::updateState, this);

  QObject::connect(box_, SIGNAL(valueChanged(int)), label_, SLOT(setNum(int)));
  QObject::connect(box_, SIGNAL(valueChanged(int)), this, SLOT(sendCallerToChatter()));
  QObject::connect(radio_walking_, SIGNAL(clicked()), this, SLOT(sendCallerToChatter()));
  QObject::connect(radio_swimming_, SIGNAL(clicked()), this, SLOT(sendCallerToChatter()));
  QObject::connect(radio_idle_, SIGNAL(clicked()), this, SLOT(sendCallerToChatter()));
  QObject::connect(this, SIGNAL(receivedString(const QString)), this, SLOT(updateLabel(const QString)));
  QObject::connect(this, SIGNAL(newState(const QString)), this, SLOT(updateRobotState(const QString)));

  //ros::Publisher talker_ = nh_.advertise<std_msgs::String>("chatter", 10);

  window_ = new QWidget;
  window_->setFixedSize(150,200);
  window_->setLayout(layout_global_);
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(window_);
  // add widget to the user interface
  context.addWidget(window_);

  //ros::spin();
}

void MyPlugin::sendCallerToChatter()
{
  /**
   * test of verbose through a ros message via publisher
   * publish to the /chatter topic the change in the GUI
   */
  QObject* obj = sender();
  talker_ = nh_.advertise<std_msgs::String>("chatter", 10);

  if (ros::ok() && talker_)
  {
    std_msgs::String msg;
    std::stringstream ss;
    if(obj == box_)
    {
      QSpinBox* box = qobject_cast<QSpinBox*>(sender());
      ss << box->property("value").toInt();
    }
    else if(obj == radio_walking_)
      ss << "Walking";
    else if(obj == radio_swimming_)
      ss << "Swimming";
    else if(obj == radio_idle_)
      ss << "Idle";
    
    msg.data = ss.str();
    talker_.publish(msg);

    ros::spinOnce();
  }
}

void MyPlugin::updateLabel(const QString msg)
{
  /**
   * update the label of the gui with a given string
   */
  label_->setText(msg);
}

void MyPlugin::updateRobotState(const QString state)
{
  /**
   * update the radio group with the new state of the robot
   */
  if(state == "Walking")
    this->radio_walking_->setChecked(true);
  else if(state == "Swimming")
    this->radio_swimming_->setChecked(true);
  else if(state == "Idle")
    this->radio_idle_->setChecked(true);
  else 
    ROS_INFO("Specified state does not exist.");
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
  talker_.shutdown();

}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_example_cpp
PLUGINLIB_EXPORT_CLASS(rqt_example_cpp::MyPlugin, rqt_gui_cpp::Plugin)
