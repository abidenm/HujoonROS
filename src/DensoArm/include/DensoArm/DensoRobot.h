// --------------------------------------------------------------------- //
// CKim - Sep. 0905 : Class encapsulating Denso Robot control and also
// Denso Robot node
// --------------------------------------------------------------------- //

// CKim - For UDP, uncomment "#define BCAP_CONNECTION_UDP" in bcap.h
// and also set robot to UDP via teaching pendant.
// Set -> F5 -> F9 -> F1 -> F1 -> UDP
// Also check set values of the variables BCAP_UDP_TIMEOUT, BCAP_UDP_RETRY ... in bCap.c
// CKim -

// CKim - ROS include
#include <ros/ros.h>

// CKim - Headers for the published / subscribed message.
// Use 'JointState' message to publish robot joint angle. rviz automatically subscribes to
// this message and visualizes the robot
#include <sensor_msgs/JointState.h>
#include <SkillMate/Mouse3dCommand.h>
#include <SkillMate/HapticCommand.h>

#include "ChunKinematics.h"

// CKim - Header and cpp provided by Denso
#include "b-Cap.h"

// CKim - Struct encapsulating Denso Robot control status
typedef struct
{
    int         iSockFD;
    uint32_t    lhController;
    uint32_t    lhTask;         // CKim - Task Handle
    uint32_t    lhRobot;        // CKim - handle to robot
    uint32_t    motorOn;
    uint32_t    slaveMode;
} RBT_DATA;

#define	SERVER_PORT_NUM			5007			// CKim - Port number (fixed)
#define SERVER_IP_ADDRESS		"192.168.1.3"	// CKim - IP Address of the robot controller
#define SERVER_TYPE				7				/* Your controller type , RC8 = 8, else is RC7 */

class DensoRobot
{

public:
    DensoRobot();
    ~DensoRobot();

    // CKim - Robot Control Functions.....
    int InitializeConnection();
    int MotorOn(bool onOff);                    // onOff: 0/1 = off/on
    int SetSlaveMode(int mode, int Async);      // Mode : 0,1,2,3 = off/p/j/t
    int MoveRobotToInit();
    void DisconnectRobot();

    // CKim - Initialize ROS publishing and subscription
    int InitializeROS();
    int ResumeSlaveMode(int errCode);


    // CKim - Access Robot Data
    int GetCurrentAngle(float pData[]);
    void GetRobotData(RBT_DATA& data)   {   data = m_robotData;     }
    void PublishJoint(float pCurr[]);

    // CKim - Kinematics functions...
    void ForwardKinematics(const Vec6& th, Mat4& M);
    void MatToZYXEuler(const Mat4& M, float p[]);

    // CKim - Convert input from mouse into target flange pos or ort. Requires current joint angle.
    void ConvertMouse(const float currJang[], const float input[], float newTgt[], int flag);

    // CKim - Convert input from haptic device into target flange pos or ort.
    // This is used to move catheter csys in x,y,z
    void ConvertHaptic2(const float delta[], float newTgt[]);

    // CKim - Convert input from haptic device into target flange pos or ort.
    // This is used to move catheter is RCM motion
    void ConvertHaptic3(const float delta[], float newTgt[]);

    void SetRefTf(const float currJang[]);

    // CKim - This function is executed in separate thread. Sets robot into slave mode
    // and continuously sends control command to the Denso robot
    static void* ThrCallback(void* pData);

private:

    // CKim - Callback for subscribed ROS message
    void mouse_3d_back(const SkillMate::Mouse3dCommand msg);
    void haptic_back(const SkillMate::HapticCommand msg);

    Vec6 m_Twists[6];     // CKim - Twist of 6 joints
    Mat4 m_InitFlange;    // CKim - Initial configuration of the flang coordinate system
    Mat4 m_InitHandle;    // CKim - Initial configuration of the handle (3D mouse)
    Mat4 m_InitCatheter;  // CKim - Initial configuration of the catheter (tip) w.r.t flange
    float m_deg2Rad;

    RBT_DATA m_robotData;

    // CKim - ROS variables
    ros::NodeHandle nh;
    ros::Subscriber mouse_sub;
    ros::Subscriber haptic_sub;
    sensor_msgs::JointState jState;
    ros::Publisher joint_pub;

    int m_controlMode;      // CKim - 0: 3D mouse, 1: HapticDevice
    Mat4 m_RefTf;           // CKim - Accumulates the pos / ort change from haptic device
    int m_prevBtn;          // CKim - Stores button input from haptic devicet

    // CKim - Values used in ThrCallBack
    static pthread_mutex_t mutex1;
    static float m_CurrJang[8];     // CKim - Current joint angle
    static float m_Tgt[8];          // CKim - Target pos/ort of the robot flange (x,y,z,Rz,Ry,Rx)

    // CKim - Variables used in subscriber callback
    float m_pMouseData[8];
    //float m_pHapticData[8];

};
