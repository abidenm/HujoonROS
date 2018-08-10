

#include "DensoRobot.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <math.h>
#include <time.h>

pthread_mutex_t DensoRobot::mutex1 = PTHREAD_MUTEX_INITIALIZER;
float DensoRobot::m_CurrJang[8] = {0};
float DensoRobot::m_Tgt[8] = {0};

DensoRobot::DensoRobot()
{
    // CKim - Initialize the Twist Coordinate xi = (v,w)
    // w is the unit length axis of the joint in base csys at zero configuration
    // q is a point on the joint in base csys at zero configuration
    Eigen::Vector3d w,q;

    w(0) = 0;       w(1) = 0;       w(2) = 1;
    q(0) = 0;       q(1) = 0;       q(2) = 1;
    m_Twists[0].block<3,1>(0,0) = -w.cross(q);    m_Twists[0].block<3,1>(3,0) = w;

    w(0) = 0;       w(1) = 1;       w(2) = 0;
    q(0) = 30;      q(1) = 0;       q(2) = 395;
    m_Twists[1].block<3,1>(0,0) = -w.cross(q);    m_Twists[1].block<3,1>(3,0) = w;

    w(0) = 0;       w(1) = 1;       w(2) = 0;
    q(0) = 30;      q(1) = 0;       q(2) = 735;
    m_Twists[2].block<3,1>(0,0) = -w.cross(q);    m_Twists[2].block<3,1>(3,0) = w;

    w(0) = 0;       w(1) = 0;       w(2) = 1;
    q(0) = 10;      q(1) = 0;       q(2) = 0;
    m_Twists[3].block<3,1>(0,0) = -w.cross(q);    m_Twists[3].block<3,1>(3,0) = w;

    w(0) = 0;       w(1) = 1;       w(2) = 0;
    q(0) = 10;      q(1) = 0;       q(2) = 1075;
    m_Twists[4].block<3,1>(0,0) = -w.cross(q);    m_Twists[4].block<3,1>(3,0) = w;

    w(0) = 0;       w(1) = 0;       w(2) = 1;
    q(0) = 10;      q(1) = 0;       q(2) = 1155;
    m_Twists[5].block<3,1>(0,0) = -w.cross(q);    m_Twists[5].block<3,1>(3,0) = w;

    m_InitFlange = Mat4::Identity();
    m_InitFlange(0,3) = 10;   m_InitFlange(1,3) = 0;    m_InitFlange(2,3) = 1155;

    // CKim - Handle is assumed to be located at the intersection of joint axis 4,5,6. It is actually mounted on the
    // surface of the link 4. COnfiguration of mouse csys w.r.t. robot base csys. at zero angle
    m_InitHandle = Mat4::Identity();
//    m_InitHandle(0,0) = 0;      m_InitHandle(1,0) = 0;      m_InitHandle(2,0) = 1;
//    m_InitHandle(0,1) = 0;     m_InitHandle(1,1) = 1;      m_InitHandle(2,1) = 0;
//    m_InitHandle(0,2) = -1;     m_InitHandle(1,2) = 0;      m_InitHandle(2,2) = 0;
//    m_InitHandle(0,3) = 10;     m_InitHandle(1,3) = 0;      m_InitHandle(2,3) = 1075;

    m_InitHandle(0,0) = 0;      m_InitHandle(1,0) = 0;      m_InitHandle(2,0) = -1;
    m_InitHandle(0,1) = 0;     m_InitHandle(1,1) = -1;      m_InitHandle(2,1) = 0;
    m_InitHandle(0,2) = -1;     m_InitHandle(1,2) = 0;      m_InitHandle(2,2) = 0;
    m_InitHandle(0,3) = 10;     m_InitHandle(1,3) = 0;      m_InitHandle(2,3) = 1075;


    // CKim - Catheter is initiall pointing in Handle is assumed to be located at the intersection of joint axis 4,5,6. It is actually mounted on the
    // surface of the link 4
    m_InitCatheter = Mat4::Identity();
    //m_InitCatheter(0,3) = 0;     m_InitCatheter(1,3) = 165;      m_InitCatheter(2,3) = 5;

    //m_InitCatheter(0,3) = 0;     m_InitCatheter(1,3) = 300;      m_InitCatheter(2,3) = 30;  // CKim - RobotCath V1
    m_InitCatheter(0,3) = 0;     m_InitCatheter(1,3) = 500;      m_InitCatheter(2,3) = 72;  // CKim - RobotCath V2
    //m_InitCatheter(0,3) = 0;     m_InitCatheter(1,3) = 360;      m_InitCatheter(2,3) = 60;

    m_deg2Rad = 3.141592/180.0;

    // CKim - Initialize robotData
    m_robotData.iSockFD = (int)NULL;            m_robotData.lhController = (uint32_t)NULL;
    m_robotData.lhRobot = (uint32_t)NULL;       m_robotData.lhTask = (uint32_t)NULL;
    m_robotData.motorOn = 0;                    m_robotData.slaveMode = 0;

   m_prevBtn = 0;
}

DensoRobot::~DensoRobot()
{

}

int DensoRobot::InitializeConnection()
{
    using namespace std;
    BCAP_HRESULT hr;            hr = BCAP_S_OK;
    BCAP_VARIANT vntOption;

    int iSockFD;            // CKim - Socket Handle
    uint32_t lhController;  // CKim - Controller handle. // u_long lhController;
    uint32_t lhTask;        // CKim - Task Handle
    uint32_t lhRobot;       // CKim - handle to robot

    // CKim - Open Network connection
    hr = bCap_Open(SERVER_IP_ADDRESS, SERVER_PORT_NUM, &iSockFD);
    //cout<<"hr = "<<setbase(16)<<"0x"<<hr<<endl;
    if FAILED(hr) {
        cerr<<"bCap_Open Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    m_robotData.iSockFD = iSockFD;
    cout<<"Opened Socket\n";

    // CKim - Turn on the controller
    hr = bCap_ServiceStart(iSockFD);
    //cout<<"hr = "<<setbase(16)<<"0x"<<hr<<endl;
    if FAILED(hr) {
        cerr<<"bCap_ServiceStart Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    cout<<"Started service...\n";


    //	CKim - Get controller handle */
    #if SERVER_TYPE > 7 /* RC8 */
        hr = bCap_ControllerConnect(iSockFD, "", "CaoProv.DENSO.VRC", "localhost", "", &lhController);
    #else /* RC7 */
        /* hr = bCap_ControllerConnect(iSockFD, "", "CaoProv.DENSO.VRC", "localhost", "", &lhController);
           It is possible to specify same parameters as RC8, becouse RC7 ignore all parameters. */
        hr = bCap_ControllerConnect(iSockFD, "", "", "", "", &lhController);
    #endif

    if FAILED(hr) {
        cerr<<"bCap_ControllerConnect Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    m_robotData.lhController = lhController;
    cout<<"Connected to controller...\n";


    // CKim - To control the robot from PC, program 'ROBSLAVE.pac' should be running in the robot controller.
    // This program can be started from teach Pendant, or by bCapTaskStart() command after setting
    // controller to 'External Auto' mode

    // CKim - Put controller to 'External Auto' mode
    vntOption.Type = VT_I2;
    vntOption.Arrays = 1;
    vntOption.Value.ShortValue = 2; // CKim - 1/2 : Internal/External Auto

    hr = bCap_ControllerExecute2(iSockFD,lhController,"PutAutoMode",&vntOption,NULL);
    if FAILED(hr) {
        cerr<<"bCap_ControllerExecute Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    cout<<"Controller set to External Auto mode...\n";

    // CKim - Get task handle and start Task 'ROBSLAVE'
    hr = bCap_ControllerGetTask(iSockFD, lhController, "RobSlave", "", &lhTask);
    if SUCCEEDED(hr){
        hr = bCap_TaskStart(iSockFD, lhTask, 1 ,(char*)"");    // Start task 1 cycle
        //cout<<"Task hr = "<<setbase(16)<<"0x"<<hr<<endl;
    }
    m_robotData.lhTask = lhTask;
    cout<<"Started Task...\n";

    // CKim - Get robot handle
    hr = bCap_ControllerGetRobot(iSockFD, lhController, "", "", &lhRobot);
    if FAILED(hr) {
        cerr<<"bCap_ControllerGetRobot Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    m_robotData.lhRobot = lhRobot;
    return 1;
}

int DensoRobot::MotorOn(bool onOff)
{
    using namespace std;
    BCAP_HRESULT hr;            hr = BCAP_S_OK;
    BCAP_VARIANT vntMotor;

    // CKim - Turn on the motor
    vntMotor.Type = VT_I2;
    vntMotor.Arrays = 1;
    vntMotor.Value.ShortValue = onOff;

    hr = bCap_RobotExecute2(m_robotData.iSockFD, m_robotData.lhRobot, "Motor", &vntMotor, NULL);

    sleep(3);

    if FAILED(hr) {
        cerr<<"bCap_RobotExecute2 Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    m_robotData.motorOn = onOff;

    return 1;
}

int DensoRobot::SetSlaveMode(int mode, int Async)
{
    using namespace std;
    BCAP_HRESULT hr;            hr = BCAP_S_OK;
    BCAP_VARIANT vntOption;

    // ------------------------------------------------------------------- //
    // CKim - Put robot into slave mode
    // "slvChangeMode", mode, option
    // mode = 0x000 = Disable, 0x001, 002, 003 = Sync P,J,T / 0x101, 102, 103 = Async P,J,T
    // Sync means that the move command will not return until the robot receives the commanded target position
    // and start moving, It takes 8 ms. If it takes longer than 8ms, timeout occurs.
    // In Async, move command returns immediately, if previous commanded target position has not been started yet. .
    // it is overwritten
    // Sync mode will return error if there is no command, ASync mode will jus stay in the last commanded position
    vntOption.Type = VT_I4;
    vntOption.Arrays = 1;

    vntOption.Value.LongValue = mode; // 0/1/2/3 = off/p/j/t
    if(Async && (mode!=0))
    {
        vntOption.Value.LongValue += 0x00000100;
    }

    hr = bCap_RobotExecute2(m_robotData.iSockFD,m_robotData.lhRobot,"slvChangeMode",&vntOption,NULL);

    if FAILED(hr) {
        cerr<<"Failed to set slave mode. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }

    if(mode!=0) {   m_robotData.slaveMode = 1;      }
    else        {   m_robotData.slaveMode = 0;      }

    return 1;
}

int DensoRobot::GetCurrentAngle(float pData[])
{
    using namespace std;
    BCAP_HRESULT hr;            hr = BCAP_S_OK;
    uint32_t lhVar;             // CKim - handle to variable

    // ------------------------------------------------------------------- //
    // CKim - Read Variable from Robot. First get handle of the variable
    // Syntax : bCap_RobotGetVariable(socket handle, robot handle, 'Name of Variable', 'pointerto handle of variable)
    // List of 'Name of variable' can be found in Section 5.3 of RC8_provider's guide
    //hr = bCap_RobotGetVariable(iSockFD, lhRobot, "@CURRENT_POSITION", "", &lhVar);
    //hr = bCap_RobotGetVariable(iSockFD, lhRobot, "@CURRENT_ANGLE", "", &lhVar);
    hr = bCap_RobotGetVariable(m_robotData.iSockFD, m_robotData.lhRobot, "@CURRENT_ANGLE", "", &lhVar);

    // CKim - Then access value through handle
    if SUCCEEDED(hr)
    {
        // CKim - Syntax : (socket handle, variable handle, pointer to the memory for storing the variable)
        // Depending on the variable you want to read, appropriate memory should be allocated in the pointer
        // For example, @CURRENT_POSITION needs 7 floats
        hr = bCap_VariableGetValue(m_robotData.iSockFD, lhVar, pData);

        if SUCCEEDED(hr){        }
        else    {
            cerr<<"bCap_VariabeGetValue Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
            return 0;
        }
    }
    else    {
        cerr<<"bCap_RobotGetVariable Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }

    // CKim - Release variable handle
    bCap_VariableRelease(m_robotData.iSockFD, lhVar);
    return 1;
}

int DensoRobot::MoveRobotToInit()
{
    using namespace ChunKinematics;
    using namespace std;
    BCAP_HRESULT hr;            hr = BCAP_S_OK;
    float fValue[7];            char moveCmd[1024];

    GetCurrentAngle(fValue);

    // ------------------------------------------------------------------- //
    // CKim - Before starting slave mode, move robot to initial position. Use @E option
    // CKim - Generate command string for moving robot. What is the format string...
    //sprintf(moveCmd,"@E P(%f,%f,%f,%f,%f,%f,%d)",200.0, -10.0, 900.0, 0.0, 0.0, 0.0, 1);
    sprintf(moveCmd,"@E J(%f,%f,%f,%f,%f,%f)",fValue[0],fValue[1],fValue[2], fValue[3],fValue[4],fValue[5]);
    do
    {
        hr = bCap_RobotMove(m_robotData.iSockFD, m_robotData.lhRobot, 1 , moveCmd, "");/* Move Robot */
        //cout<<"hr = "<<setbase(16)<<"0x"<<hr<<endl;
    } while ((hr == BCAP_E_ROBOTISBUSY) || (hr == BCAP_E_UNAVAILABLE));


    // CKim - Reset the curren joint angle and current target position variable
    Vec6 jA;    Mat4 M = Mat4::Identity();
    for(int i=0; i<6; i++)  {   m_CurrJang[i] = fValue[i];  jA[i] = fValue[i]*m_deg2Rad;    }
    ForwardKinematics(jA,M);
    MatToZYXEuler(M,m_Tgt);
    for(int i=3; i<6; i++) {   m_Tgt[i]*=(180.0/3.141592);  }

    cout<<"Motion complete...\n";
    usleep(100000);

    // ------------------------------------------------------------------- //
}

int DensoRobot::InitializeROS()
{
    jState.name.resize(6);      jState.position.resize(6);

    // CKim - Initialize ROS Subscription
    mouse_sub = nh.subscribe("mouse_3d", 100, &DensoRobot::mouse_3d_back, this);
    haptic_sub = nh.subscribe("HapticCmd", 100, &DensoRobot::haptic_back,this);
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void DensoRobot::DisconnectRobot()
{
    using namespace std;
    BCAP_HRESULT hr = BCAP_S_OK;
    cout<<"Exiting on Error...\n";

    if(m_robotData.slaveMode)
    {
        if(!SetSlaveMode(0,0))
        {
            cerr<<"Error : Slave Mode Exit"<<hr<<endl;
        }
        else    {   cout<<"Robot exiting slave mode"<<endl;        }
    }

    // CKim - Turn off the motor
    if(m_robotData.motorOn)
    {
        if(! MotorOn(0))
        {
            cerr<<"Error : Turning off the motor"<<endl;
        }
        else    {   cout<<"Turning off the motor"<<endl;    }
        sleep(1);
    }

    // CKim - Release robot handle
    if(m_robotData.lhRobot)
    {
        hr = bCap_RobotRelease(m_robotData.iSockFD, m_robotData.lhRobot);
        if FAILED(hr) {
            cerr<<"Error : Releasing robot Handle. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;        }
        else    {   cout<<"Robot released"<<endl;   }
    }

    // CKim - Stop Task
    if(m_robotData.lhTask)
    {
        hr = bCap_TaskStop(m_robotData.iSockFD,m_robotData.lhTask,1,(char*)"");
        if FAILED(hr) {
            cerr<<"Error : Stopping Task. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;        }
        else    {   cout<<"Task Stopped"<<endl;     }
    }    // 1 : suspend

    // CKim - Release controller handle
    if(m_robotData.lhController)
    {
       hr = bCap_ControllerDisconnect(m_robotData.iSockFD, m_robotData.lhController);
       if FAILED(hr) {
           cerr<<"Error : Disconnecting Controller. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;        }
       else {   cout<<"Controller disconnected"<<endl;  }
    }

    /* Stop b-CAP service (Very important in UDP/IP connection) */
    if(m_robotData.iSockFD)
    {
        hr = bCap_ServiceStop(m_robotData.iSockFD);
        if FAILED(hr) {
            cerr<<"Error : Stopping Service. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;        }
        else    {   cout<<"Service Stopped"<<endl;  }

        // CKim - Close socket
        bCap_Close(m_robotData.iSockFD);
        cout<<"Socket closed"<<endl;
    }
}

void DensoRobot::ForwardKinematics(const Vec6& th, Mat4& M)
{
    // CKim - Calculate Exp(xi*th)*InitTool
    M = Mat4::Identity();

    for(int i=0; i<6; i++)
    {
        M = M*ChunKinematics::TwistExp(m_Twists[i],th(i));
    }
    M = M*m_InitFlange;
}

void DensoRobot::MatToZYXEuler(const Mat4& M, float p[])
{
    for(int i=0; i<3; i++)  {   p[i] = M(i,3);  }
    double rz = atan2(M(1,0),M(0,0));
    double ry = asin(-M(2,0));
    double rx = atan2(M(2,1),M(2,2));
    p[3] = rx;  p[4] = ry;  p[5] = rz;
}

void DensoRobot::ConvertMouse(const float currJang[], const float input[], float newTgt[], int flag)
{
    using namespace ChunKinematics;

    Vec6 newth, th;         Eigen::Vector3d dh;
    for(int i=0; i<6; i++)   {   newth(i) = th(i) = currJang[i]*m_deg2Rad;  }

    Mat4 tmp;           tmp = Mat4::Identity();
    Mat4 TgtM;          ForwardKinematics(th,TgtM);

    if(flag)    // CKim - Orientation control
    {
        float j3 = input[3]*3.141592/180.0;     newth(3)-=j3;
        float j4 = input[4]*3.141592/180.0;     newth(4)+=j4;
        float j5 = input[5]*3.141592/180.0;     newth(5)-=j5;
        ForwardKinematics(newth,tmp);
        TgtM.block<3,3>(0,0) = tmp.block<3,3>(0,0);
    }
    else        // CKim - Translation control
    {
        for(int i=0; i<3; i++)  {   dh(i) = input[i];     }

        Mat4 Xf = InvTf(m_InitFlange)*TwistExp(m_Twists[5],-th(5))*TwistExp(m_Twists[4],-th(4))*m_InitHandle;
        tmp.block<3,1>(0,3) = Xf.block<3,3>(0,0)*dh;
        TgtM *= tmp;
    }

    MatToZYXEuler(TgtM,newTgt);
    for(int i=3; i<6; i++) {   newTgt[i]*=(180.0/3.141592);  }
}

void DensoRobot::ConvertHaptic2(const float dt[], float newTgt[])
{
    using namespace ChunKinematics;

    // ------------------------------------
    // CKim - Process Haptic Device input. Apply coordinate transformation, deadband and scale.
    // Input from SkillMate. 0-2 delta position. 3,4 -1 to 1 like joystick
    float posScl = 1.0;    float rotScl = 1.0;
    float llim = 0;        float ulim = 4.0;
    float val[6];          float sgn = 1.0;     float delta[6];

    val[0] = dt[0];     val[1] = -dt[2];    val[2] = dt[1];
    val[3] = dt[5];     val[4] = dt[4];     val[5] = dt[3];

    for(int i=0; i<3; i++)  {
        val[i] *= posScl;       sgn = fabs(val[i])/val[i];
        if(fabs(val[i]) > llim) {
            if(fabs(val[i]) < ulim) {
                delta[i] = val[i];   }
            else    {   delta[i] = ulim*sgn;   }
        }
        else    {       delta[i] = 0;                     }
    }

    for(int i=3; i<6; i++)  {
        val[i] *= rotScl;   sgn = fabs(val[i])/val[i];
        if(fabs(val[i]) > llim) {
            if(fabs(val[i]) < ulim) {
                delta[i] = val[i];   }
            else    {   delta[i] = ulim*sgn;   }
        }
        else    {       delta[i] = 0;                     }
    }
    delta[4] = 0;


    // ------------------------------------------------------

    Mat4 M = m_RefTf;

    // CKim - Calcuate configuration change
    Mat4 dG = Mat4::Identity();     Mat3 Rx, Ry, Rz;
    RotX(Rx,delta[3]*m_deg2Rad);    RotZ(Rz,delta[4]*m_deg2Rad);      RotY(Ry,delta[5]*m_deg2Rad);
    dG.block<3,3>(0,0) = Rx*Rz*Ry;
    for(int i=0; i<3; i++)  {   dG(i,3) = delta[i]; }

    // CKim - Accumulate desired configuration change
    m_RefTf = M*dG;
    Mat4 TgtM = m_RefTf*InvTf(m_InitCatheter);
    MatToZYXEuler(TgtM,newTgt);
    for(int i=3; i<6; i++) {   newTgt[i]*=(180.0/3.141592);  }
}

void DensoRobot::ConvertHaptic3(const float dt[], float newTgt[])
{
    using namespace ChunKinematics;

    // ------------------------------------
    // CKim - Process Haptic Device input. Apply coordinate transformation, deadband and scale.
    // Input from SkillMate. 0-2 delta position. 3,4 -1 to 1 like joystick.
    // Here we only use dt 0-2
    float Scl[3] = { 0.1, 0.1, 1.0 };       float llim = 0;        float ulim = 4.0;     float sgn = 1.0;
    float delta[3];        float val[3];

    // CKim - Conver dx, dy of haptic device into rotation about z and x in cathter csys.
    // dx of haptic input corresponds to positive rotation in z
    // dy or haptic input corresponds to negative rotation in y
    val[0] = dt[0];     val[1] = -dt[1];    val[2] = -dt[2];

    for(int i=0; i<3; i++)  {
        val[i] *= Scl[i];       sgn = fabs(val[i])/val[i];
        if(fabs(val[i]) > llim) {
            if(fabs(val[i]) < ulim) {
                delta[i] = val[i];   }
            else    {   delta[i] = ulim*sgn;   }
        }
        else    {       delta[i] = 0;                     }
    }


    // ------------------------------------
    // CKim - Convert transformed delta to transformation
    Mat4 M = m_RefTf;

    // CKim - Calcuate configuration change. delta 0,1,2 is Rz, Rx dy in catheter csys
    Mat4 dG = Mat4::Identity();     Mat3 Rx, Rz;
    Mat4 G1 = Mat4::Identity();     RotZ(Rz,delta[0]*m_deg2Rad);    G1.block<3,3>(0,0) = Rz;
    Mat4 G2 = Mat4::Identity();     RotX(Rx,delta[1]*m_deg2Rad);    G2.block<3,3>(0,0) = Rx;
    Mat4 G3 = Mat4::Identity();     G3(1,3) = delta[2];
    dG = G1*G2*G3;
    //dG(1,3) = delta[2];

    // CKim - Accumulate desired configuration change
    m_RefTf = M*dG;
    Mat4 TgtM = m_RefTf*InvTf(m_InitCatheter);
    MatToZYXEuler(TgtM,newTgt);
    for(int i=3; i<6; i++) {   newTgt[i]*=(180.0/3.141592);  }
}

void DensoRobot::SetRefTf(const float currJang[])
{
    using namespace ChunKinematics;

    // CKim - Calculate the configuration of current tool coordinate system
    Mat4 M = Mat4::Identity();

    for(int i=0; i<6; i++)
    {
        M = M*TwistExp(m_Twists[i],currJang[i]*m_deg2Rad);
    }
    m_RefTf = M*m_InitFlange*m_InitCatheter;
}

void* DensoRobot::ThrCallback(void* pData)
{
    using namespace std;
    DensoRobot* pRobot = (DensoRobot*)pData;        BCAP_HRESULT hr = BCAP_S_OK;
    RBT_DATA rData;
    int cnt = 0;    int res;

    // CKim - Current joint angle, Target robot pos/ort. Increment of the pos/ort, from mouse or haptic input
    float pCurr[8], pTgt[8];    float delta[6];

    // CKim - Initialize varaibles. Update current location,
    pRobot->GetCurrentAngle(pCurr);
    pRobot->GetRobotData(rData);
    for(int i=0; i<6; i++)  {   delta[i] = 0;   }
    pRobot->ConvertMouse(pCurr,delta,m_Tgt,0);

    pthread_mutex_lock( &mutex1 );
    for(int i=0; i<8; i++)  {   pTgt[i] = m_Tgt[i]; m_CurrJang[i] = pCurr[i]; }
    pthread_mutex_unlock( &mutex1 );
    pthread_yield();


    // CKim - Start slave mode
    cout<<"Starting slave mode...\n";
    //res = pRobot->SetSlaveMode(1,1);    // P, Async
    res = pRobot->SetSlaveMode(1,0);    // P, Sync
    if(!res)    {
        printf("Failed to start slave mode\n");
        pRobot->DisconnectRobot();
        ros::shutdown();
        return 0;
    }
    ROS_INFO("Robot entered slave mode, press 'ctrl+C' to stop\n");

    while(ros::ok())
    {
        // CKim - Command robot to desired end effector pos/ort
        // In Async mode, this line executes in 2000 Hz.
        // In Sync mode, this line executes in 125 Hz.
        hr = bCap_RobotExecuteSlaveMove(rData.iSockFD,rData.lhRobot,(char*)"slvMove",pTgt,pCurr);

        if FAILED(hr) {
            cerr<<"bCap_RobotExecuteSlaveMove Error Code = "<<setbase(16)<<"0x"<<hr<<endl;

            ROS_INFO("Slave motion error. Check error message on pendant. Then press any key to resume slave mode. Press 'q' to quit\n");
            if( 'q' == getchar() )
            {
                ros::shutdown();
                break;
            }
            else
            {
                if(!pRobot->ResumeSlaveMode(hr))
                {
                    ROS_INFO("Failed to resume\n");
                    ros::shutdown();
                    break;
                }
            }
        }

        // CKim - Update current target and angle
        pthread_mutex_lock( &mutex1 );
        for(int i=0; i<8; i++)  {   pTgt[i] = m_Tgt[i]; m_CurrJang[i] = pCurr[i]; }
        pthread_mutex_unlock( &mutex1 );

        // CKim - Publish robot joint angle every once in a while
        // http://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher
        cnt++;
        //if(cnt==10)
        if(cnt==2)
        {
            pRobot->PublishJoint(pCurr);
            cnt = 0;
        }

    }
    ROS_INFO("Terminating slave thread...\n");
    pRobot->DisconnectRobot();
}

void DensoRobot::mouse_3d_back(const SkillMate::Mouse3dCommand msg)
{
    float magpos = 5.0;    float magort = 3.0;
    float db = 30;         float max = 350.0;
    float val[6];          float sgn = 1.0;

    // CKim - Check button press
    if(msg.type == 2)   // CKim - 1. Motion, 2. Button press event
    {
        for(int i=0; i<6; i++)  {   m_pMouseData[i] = 0;  }
        if(msg.button == 0 && msg.press == 1)   {   m_pMouseData[6] = 1;  m_controlMode = 0;    }
        if(msg.button == 1 && msg.press == 1)   {   m_pMouseData[6] = 0;  m_controlMode = 0;    }
        return;
    }

    // CKim - Process mouse input. Apply coordinate transformation, deadband and scale.
    // Input from 3D mouse ranges from -350 to 350 for each axis.
    val[0] = msg.x;     val[1] = -msg.y;    val[2] = msg.z;
    val[3] = msg.rx;    val[4] = msg.ry;    val[5] = msg.rz;

    for(int i=0; i<3; i++) {
        if(fabs(val[i]) > db) {
            sgn = fabs(val[i])/val[i];
            m_pMouseData[i] = magpos/max*(val[i] - sgn*db);
        }
        else    {   m_pMouseData[i] = 0;    }
    }

    for(int i=3; i<6; i++) {
        if(fabs(val[i]) > db) {
            sgn = fabs(val[i])/val[i];
            m_pMouseData[i] = magort/max*(val[i] - sgn*db);
        }
        else    {   m_pMouseData[i] = 0;    }
    }


    // CKim - Convert Mouse Input into robot end effector command and update target
    if(m_controlMode == 0)    // CKim - 3D Mouse
    {
        pthread_mutex_lock( &mutex1 );
        ConvertMouse(m_CurrJang,m_pMouseData,m_Tgt,m_pMouseData[6]);
        pthread_mutex_unlock( &mutex1 );
    }
}

void DensoRobot::haptic_back(const SkillMate::HapticCommand cmd)
{
    // CKim - Check button press.
    if(cmd.btn[0] == 1)
    {
        m_controlMode = 1;
        if(m_prevBtn == 0)
        {
            pthread_mutex_lock( &mutex1 );
            SetRefTf(m_CurrJang);
            pthread_mutex_unlock( &mutex1 );
            m_prevBtn = 1;
            return;
        }
    }
    else
    {
        m_controlMode = 0;
        m_prevBtn = 0;
        return;
    }

    // CKim - Control using Haptic Command
    if(m_controlMode == 1)
    {
        float val[6];
        for(int i=0; i<6; i++)  {   val[i] = cmd.array[i];  }

        // CKim - Convert haptic input into traget position
        pthread_mutex_lock( &mutex1 );
        ConvertHaptic3(val,m_Tgt);
        //ConvertHaptic2(val,m_Tgt);
        pthread_mutex_unlock( &mutex1 );
    }

}

void DensoRobot::PublishJoint(float pCurr[])
{
    jState.header.stamp = ros::Time::now();
    jState.name[0] = "J1";  jState.position[0] = pCurr[0]*3.141592/180.0;
    jState.name[1] = "J2";  jState.position[1] = pCurr[1]*3.141592/180.0;
    jState.name[2] = "J3";  jState.position[2] = pCurr[2]*3.141592/180.0;
    jState.name[3] = "J4";  jState.position[3] = pCurr[3]*3.141592/180.0;
    jState.name[4] = "J5";  jState.position[4] = pCurr[4]*3.141592/180.0;
    jState.name[5] = "J6";  jState.position[5] = pCurr[5]*3.141592/180.0;
    joint_pub.publish(jState);
}

int DensoRobot::ResumeSlaveMode(int errCode)
{
    using namespace std;
    BCAP_HRESULT hr = BCAP_S_OK;

    // CKim - Stop Slave mode
    if(!SetSlaveMode(0,0))  {   cerr<<"Failed to terminate slave mode\n";  return 0;   };

    // CKim - Clear Error
    BCAP_VARIANT vntOption;
    vntOption.Type = VT_I4;
    vntOption.Arrays = 1;
    vntOption.Value.ShortValue = errCode; // CKim - 1/2 : Internal/External Auto

    hr = bCap_ControllerExecute2(m_robotData.iSockFD,m_robotData.lhController,"ClearError",&vntOption,NULL);
    sleep(3);
    if FAILED(hr) {
        cerr<<"Failed to clear error. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    cout<<"Cleared Error. Now resuming slave mode\n";


    // CKim - Put controller bakc to 'External Auto' mode
    vntOption.Type = VT_I2;
    vntOption.Arrays = 1;
    vntOption.Value.ShortValue = 2; // CKim - 1/2 : Internal/External Auto

    hr = bCap_ControllerExecute2(m_robotData.iSockFD,m_robotData.lhController,"PutAutoMode",&vntOption,NULL);
    if FAILED(hr) {
        cerr<<"bCap_ControllerExecute Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    cout<<"Controller set back to External Auto mode...\n";

    // CKim - Restart Task.
    hr = bCap_ControllerGetTask(m_robotData.iSockFD, m_robotData.lhController, "RobSlave", "", &m_robotData.lhTask);
    if SUCCEEDED(hr){
        hr = bCap_TaskStart(m_robotData.iSockFD, m_robotData.lhTask, 1 ,(char*)"");    // Start task 1 cycle
        //cout<<"Task hr = "<<setbase(16)<<"0x"<<hr<<endl;
    }
    else {
        cerr<<"Failed to resume task. Error Code = "<<setbase(16)<<"0x"<<hr<<endl;
        return 0;
    }
    cout<<"Task resumed...\n";

    // CKim - Turn on motor. Restart
    if(!MotorOn(1)) {   cerr<<"Failed to restart motor\n";  return 0;   }
    cout<<"Motor turned on, moving to initial position..\n";

    MoveRobotToInit();

    if(!SetSlaveMode(1,0))  {   cerr<<"Failed to restart slave mode\n";     return 0;   }
    return 1;
}
