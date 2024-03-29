#ifndef RQT_joy2dynamixel_MY_PLUGIN_H
#define RQT_joy2dynamixel_MY_PLUGIN_H

#include "ros/ros.h"
#include <rqt_gui_cpp/plugin.h>
#include <rqt_joy2dynamixel/ui_my_plugin.h>
#include <std_msgs/Int16.h>
#include <QWidget>
#include <QtWidgets>


namespace rqt_joy2dynamixel
{

    enum Range {INI = 600, MIN = 0, MAX = 2000};

    class MyPlugin : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        MyPlugin();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
            qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
            const qt_gui_cpp::Settings& instance_settings);

        // Comment in to signal that the plugin has a way to configure it
        // bool hasConfiguration() const;
        // void triggerConfiguration();
    private Q_SLOTS:
        void updateAmplitude(int val);

    private:
        Ui::MyPluginWidget ui_;
        QWidget* window_;
        QSpinBox* box_;
        QSlider* slider_;

        ros::NodeHandle nh_;
        ros::Publisher amplitude_pub_;
    };
}  // namespace rqt_joy2dynamixel
#endif  // RQT_joy2dynamixel_MY_PLUGIN_H
