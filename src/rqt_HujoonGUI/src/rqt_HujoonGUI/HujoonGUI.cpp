//#include "my_plugin.h"
#include <rqt_HujoonGUI/HujoonGUI.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QDoubleSpinBox>
#include <QScrollBar>

namespace rqt_HujoonGUI {

HujoonGUI::HujoonGUI()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("HujoonGUI");

  // CKim - Call this function to be able to passe QVector<float>
  // over signal and slot functions
  qRegisterMetaType< QVector<float> >("QVector<float>");
}

void HujoonGUI::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // CKim - Initialize ROS and variables
  m_PosTension.resize(12);

  cath_sub = getNodeHandle().subscribe("/catheter_states", 100, &HujoonGUI::callbackCath,this);
  gui_pub = getNodeHandle().advertise<RobotCatheter::catheterState>("/cath_gui_cmd",1);

  // CKim - Connect Signals and Slots
  QObject::connect(this,SIGNAL(newValue(QVector<float>)),this,SLOT(UpdateGUI(QVector<float>)));
  connect(ui_.Btn_Action,SIGNAL(clicked()),this,SLOT(SendCmd()));
  connect(ui_.hzScrl_LED,SIGNAL(valueChanged(int)),this,SLOT(SetLED(int)));

  // CKim - Set UI
  ui_.hzScrl_LED->setRange(0,4999);      ui_.hzScrl_LED->setValue(0);    ui_.hzScrl_LED->setSingleStep(1);

//  ui_.dblSpin_T0->setRange(0,65535);      ui_.dblSpin_T0->setReadOnly(1);
//  ui_.dblSpin_T1->setRange(0,65535);      ui_.dblSpin_T1->setReadOnly(1);
//  ui_.dblSpin_T2->setRange(0,65535);      ui_.dblSpin_T2->setReadOnly(1);
//  ui_.dblSpin_T3->setRange(0,65535);      ui_.dblSpin_T3->setReadOnly(1);

//  // 315,915, 0
//  ui_.dblSpin_P0->setRange(255,1600000);    ui_.dblSpin_P0->setDecimals(0);     ui_.dblSpin_P0->setReadOnly(1);
//  ui_.dblSpin_P1->setRange(255,1600000);    ui_.dblSpin_P1->setDecimals(0);     ui_.dblSpin_P1->setReadOnly(1);
//  ui_.dblSpin_P2->setRange(255,1600000);    ui_.dblSpin_P2->setDecimals(0);     ui_.dblSpin_P2->setReadOnly(1);
//  ui_.dblSpin_P3->setRange(255,1600000);    ui_.dblSpin_P3->setDecimals(0);     ui_.dblSpin_P3->setReadOnly(1);

  // CKim - Setup Table
  ui_.table_motorstate->setItem(0,1,new	QTableWidgetItem(QString("MOTOR 1")));
  ui_.table_motorstate->setItem(0,2,new	QTableWidgetItem(QString("MOTOR 2")));
  ui_.table_motorstate->setItem(0,3,new	QTableWidgetItem(QString("MOTOR 3")));
  ui_.table_motorstate->setItem(0,4,new	QTableWidgetItem(QString("MOTOR 4")));

  ui_.table_motorstate->setItem(1,0,new	QTableWidgetItem(QString("POSITION (mm)")));
  ui_.table_motorstate->setItem(2,0,new	QTableWidgetItem(QString("LOAD")));
  ui_.table_motorstate->setItem(3,0,new	QTableWidgetItem(QString("CURRENT")));

  ui_.table_motorstate->setColumnWidth(0, 120);
  for(int i=1; i<5; i++)    ui_.table_motorstate->setColumnWidth(i, 80);

}

void HujoonGUI::shutdownPlugin()
{
  // TODO unregister all publishers here
    //cath_sub.shutdown();
}

void HujoonGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void HujoonGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void HujoonGUI::callbackCath(const RobotCatheter::catheterState msg)
{
    // CKim - Upon receiving ROS message, emit signal and pass value
    //ROS_INFO("Sparta!!");
    for(int i=0; i<4; i++)  {
        m_PosTension[i] = msg.Displacement[i];
        m_PosTension[i+4] = msg.Tension[i];
        m_PosTension[i+8] = msg.Current[i];
    }
    emit newValue(m_PosTension);
}

void HujoonGUI::UpdateGUI(QVector<float> val)
{
    //ROS_INFO("Sparta!!");

    QString qstr1, qstr2, qstr3;
    for(int i=0; i<4; i++)
    {
        qstr1.sprintf("%.2f",val[i]);         ui_.table_motorstate->setItem(1,i + 1, new QTableWidgetItem(qstr1));
        qstr2.sprintf("%.2f",val[i+4]);       ui_.table_motorstate->setItem(2,i + 1, new QTableWidgetItem(qstr2));
        qstr3.sprintf("%.2f",val[i+8]);       ui_.table_motorstate->setItem(3,i + 1, new QTableWidgetItem(qstr3));

        //        ui_.table_motorstate->setItem(1,i + 1,new	QTableWidgetItem(QString::number(val[i])));
//        ui_.table_motorstate->setItem(2,i + 1,new	QTableWidgetItem(QString::number(val[i+4])));
//        ui_.table_motorstate->setItem(3,i + 1,new	QTableWidgetItem(QString::number(val[i+8])));
    }
}

void HujoonGUI::SendCmd()
{
    // CKim - Publish commands
//    //ROS_INFO("Sparta!!");
//    denso::catheterState msg;
//    msg.Displacement[0] = 615;      msg.Displacement[1] = 615;
//    msg.Displacement[2] = 615;      msg.Displacement[3] = 615;
//    msg.LightPower = 255-ui_.hzScrl_LED->value();
//    gui_pub.publish(msg);
}

void HujoonGUI::SetLED(int val)
{
    // CKim - Publish commands
    //ROS_INFO("Sparta!!");
    RobotCatheter::catheterState msg;
    msg.Displacement[0] = 615;      msg.Displacement[1] = 615;
    msg.Displacement[2] = 615;      msg.Displacement[3] = 615;
    msg.LightPower = val;
    gui_pub.publish(msg);
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
//PLUGINLIB_DECLARE_CLASS(my_namespace, MyPlugin, my_namespace::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_DECLARE_CLASS(rqt_HujoonGUI, HujoonGUI, rqt_HujoonGUI::HujoonGUI, rqt_gui_cpp::Plugin)
