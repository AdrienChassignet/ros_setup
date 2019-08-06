#include "rqt_joy2dynamixel/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_joy2dynamixel
{

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

  QLabel* label = new QLabel("       Amplitude\n[1 count = 0.088°]");
  box_ = new QSpinBox();
  box_->setMaximumWidth(50);
  box_->setRange(MIN, MAX);
  box_->setValue(INI);
  slider_ = new QSlider(Qt::Vertical);
  slider_->setMinimumHeight(80);
  slider_->setRange(MIN, MAX);
  slider_->setSliderPosition(INI);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(label);
  layout->setAlignment(label, Qt::AlignHCenter);
  layout->addWidget(box_);
  layout->setAlignment(box_, Qt::AlignHCenter);
  layout->addWidget(slider_);
  layout->setAlignment(slider_, Qt::AlignHCenter);

  QObject::connect(box_, SIGNAL(valueChanged(int)), slider_, SLOT(setValue(int)));
  QObject::connect(slider_, SIGNAL(valueChanged(int)), box_, SLOT(setValue(int)));
  QObject::connect(box_, SIGNAL(valueChanged(int)), this, SLOT(updateAmplitude(int)));
  QObject::connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(updateAmplitude(int)));
  

  amplitude_pub_ = nh_.advertise<std_msgs::Int16>("joy2dynamixel/amplitude", 10);

  // create QWidget
  window_ = new QWidget;
 // window_->setFixedSize(150,300); //it may makes rqt crash
  window_->setLayout(layout);
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(window_);
  // add widget to the user interface
  context.addWidget(window_);

  //ros::spin();
}

void MyPlugin::updateAmplitude(int val)
{
  /**
   * Publish new amplitude on the /joy2dynamixel/amplitude topic
   * according to the updated value in the GUI
   */
  if(ros::ok() && amplitude_pub_)
  {
    std_msgs::Int16 amp;
    amp.data = val;
    amplitude_pub_.publish(amp);
    ros::spinOnce();
  }
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
  amplitude_pub_.shutdown();

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

}  // namespace rqt_joy2dynamixel
PLUGINLIB_EXPORT_CLASS(rqt_joy2dynamixel::MyPlugin, rqt_gui_cpp::Plugin)
