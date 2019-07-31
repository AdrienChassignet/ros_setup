#include "rqt_turtle_teleop/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_turtle_teleop {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , window_(0)
  , linear_max_speed_(LIN_INI)
  , angular_max_speed_(ANG_INI)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  
  QLabel* label1 = new QLabel("Linear speed");
  box1_ = new QSpinBox();
  box1_->setMaximumWidth(50);
  box1_->setRange(LIN_MIN, LIN_MAX);
  box1_->setValue(LIN_INI);
  slider1_ = new QSlider(Qt::Vertical);
  slider1_->setMinimumHeight(80);
  slider1_->setRange(LIN_MIN, LIN_MAX);
  slider1_->setSliderPosition(LIN_INI);
  QLabel* label2 = new QLabel("Angular speed");
  box2_ = new QSpinBox();
  box2_->setMaximumWidth(50);
  box2_->setRange(ANG_MIN, ANG_MAX);
  box2_->setValue(ANG_INI);
  slider2_ = new QSlider(Qt::Vertical);
  slider2_->setMinimumHeight(80);
  slider2_->setRange(ANG_MIN, ANG_MAX);
  slider2_->setSliderPosition(ANG_INI);

  QVBoxLayout* layout1 = new QVBoxLayout();
  layout1->addWidget(label1);
  layout1->setAlignment(label1, Qt::AlignHCenter);
  layout1->addWidget(box1_);
  layout1->setAlignment(box1_, Qt::AlignHCenter);
  layout1->addWidget(slider1_);
  layout1->setAlignment(slider1_, Qt::AlignHCenter);

  QVBoxLayout* layout2 = new QVBoxLayout();
  layout2->addWidget(label2);
  layout2->setAlignment(label2, Qt::AlignHCenter);
  layout2->addWidget(box2_);
  layout2->setAlignment(box2_, Qt::AlignHCenter);
  layout2->addWidget(slider2_);
  layout2->setAlignment(slider2_, Qt::AlignHCenter);

  QGridLayout* layout = new QGridLayout();
  layout->addLayout(layout1, 0 ,0);
  layout->addLayout(layout2, 0, 1);

  QObject::connect(box1_, SIGNAL(valueChanged(int)), slider1_, SLOT(setValue(int)));
  QObject::connect(slider1_, SIGNAL(valueChanged(int)), box1_, SLOT(setValue(int)));
  QObject::connect(box2_, SIGNAL(valueChanged(int)), slider2_, SLOT(setValue(int)));
  QObject::connect(slider2_, SIGNAL(valueChanged(int)), box2_, SLOT(setValue(int)));
  QObject::connect(box1_, SIGNAL(valueChanged(int)), this, SLOT(updateMaxSpeed(int)));
  QObject::connect(slider1_, SIGNAL(valueChanged(int)), this, SLOT(updateMaxSpeed(int)));
  QObject::connect(box2_, SIGNAL(valueChanged(int)), this, SLOT(updateMaxSpeed(int)));
  QObject::connect(slider2_, SIGNAL(valueChanged(int)), this, SLOT(updateMaxSpeed(int)));

  max_speed_pub_ = nh_.advertise<rqt_turtle_teleop::MaxSpeed>("scaleSpeed", 10);
  // change_max_sub_ = nh_.subscribe("changeScaleSpeed", 10, &MyPlugin::changeMaxCallback, this);

  // create QWidget
  window_ = new QWidget();
  //window_->setFixedSize(300,400);
  window_->setLayout(layout);

  // extend the widget with all attributes and children from UI file
  ui_.setupUi(window_);
  // add widget to the user interface
  context.addWidget(window_);
}

void MyPlugin::updateMaxSpeed(int val)
{
  /**
   * Publish new scale factors on the /scaleSpeed topic according
   * to the updated value in the GUI
   */
  QObject* obj = sender();
  if(ros::ok() && max_speed_pub_)
  {
    rqt_turtle_teleop::MaxSpeed scale;

    if(obj == box1_ || obj == slider1_)
    {
      linear_max_speed_ = val;
    }
    else if(obj == box2_ || obj == slider2_)
    {
      angular_max_speed_ = val;
    }

    scale.linear_max_speed = (double) linear_max_speed_;
    scale.angular_max_speed = (double) angular_max_speed_;

    max_speed_pub_.publish(scale);
    ros::spinOnce();
  }
}

// void MyPlugin::changeMaxCallback(const rqt_turtle_teleop::MaxSpeed::ConstPtr& val)
// {
//   linear_max_speed_ += (int) ((val->linear_max_speed))/1000;
//   angular_max_speed_ += (int) ((val->angular_max_speed))/1000;

//   updateMaxSpeed(0);
// }

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
  max_speed_pub_.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
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

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_turtle_teleop::MyPlugin, rqt_gui_cpp::Plugin)
