//#ifndef my_namespace__my_plugin_H
//#define my_namespace__my_plugin_H

#ifndef rqt_HujoonGUI__HujoonGUI_H
#define rqt_HujoonGUI__HujoonGUI_H

// CKim - ROS include
#include <ros/ros.h>

// CKim - Include for rqt cpp GUI
#include <rqt_gui_cpp/plugin.h>
//#include <ui_my_plugin.h>
#include <ui_HujoonWidget.h>
#include <QWidget>

// CKim - Additional headers for QT and ROS messages
#include <QVector>
#include <QMetaType>
#include <RobotCatheter/catheterState.h>

namespace rqt_HujoonGUI {

class HujoonGUI
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  HujoonGUI();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

// CKim - Signals and Slots for communicating with the ROS
// Emit a 'signal' from the callback function when subscribed ROS message is received
// and 'connect' a 'slot' to handle the signal
public slots:
    void UpdateGUI(QVector<float>); // CKim - Connects to signal emitted from ROS message callback
    void SendCmd();                 // CKim - Connects to button click
    void SetLED(int val);

signals:
    void newValue(QVector<float>);  // CKim -Emitted from ROS message callback


private:
  //Ui::MyPluginWidget ui_;
  Ui::HujoonGUIWidget ui_;      // CKim - Ui::"ObjectName in .ui file"
  QWidget* widget_;

  // CKim - Additional variables for ROS
  ros::Subscriber cath_sub;     // CKim - Subscribes to Catheter Robot States
  ros::Publisher gui_pub;       // CKim - Publishes user input
  void callbackCath(const RobotCatheter::catheterState msg);

  QVector<float> m_PosTension;

};
} // namespace
#endif // my_namespace__my_plugin_H
