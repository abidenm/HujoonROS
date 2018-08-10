// --------------------------------------------------------------------- //
// CKim - Sep. 0905 : Source code for the denso_rt.exe
// which is a ROS node for controlling Denso robot
// Actual codes are in DensoRobot class (RbtClass.cpp)
// --------------------------------------------------------------------- //

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include "b-Cap.h"
#include "DensoRobot.h"

//ROS include//
#include <ros/ros.h>

using namespace std;

//float maxdiff = -100000;

/* reads from keypress, doesn't echo */
int getch(void);

int main(int argc, char **argv)
{
    // CKim - Initialize ROS
    ros::init(argc, argv, "Denso");

    // CKim - DensoRobot class contains actual code for bCap communications
    int res;                float fValue[7];              int iret1;
    DensoRobot dRobot;      pthread_t slaveThread;


    ROS_INFO("Starting Robot Node....\n");
    sleep(2);

    // ------------------------------------------------------------------- //
    ROS_INFO("Press any key to Connect to Robot.\n");
    getchar();
    res = dRobot.InitializeConnection();
    if(!res)    {
        ROS_INFO("Failed to initialize the robot\n");
        dRobot.DisconnectRobot();
        return 0;
    }
    ROS_INFO("Robot Initialized.\n");
    // ------------------------------------------------------------------- //

    // ------------------------------------------------------------------- //
    ROS_INFO("Press any key to Turn on the Robot.\n");
    getchar();

    res = dRobot.MotorOn(1);
    if(!res)    {
        ROS_INFO("Failed to turn on the robot\n");
        dRobot.DisconnectRobot();
        return 0;
    }
    ROS_INFO("Robot turned on.\n");
    // ------------------------------------------------------------------- //

    // ------------------------------------------------------------------- //
    res = dRobot.GetCurrentAngle(fValue);
    if(!res)    {
        ROS_INFO("Failed to read joint angle\n");
        dRobot.DisconnectRobot();
        return 0;
    }
    for (int i = 0; i < 6;i++){		printf("@CurJang:%d %f \n", i, fValue[i]); }

    ROS_INFO("Press any key to Move robot to initial position.\n");
    getchar();
    dRobot.MoveRobotToInit();

    res = dRobot.GetCurrentAngle(fValue);
    if(!res)    {
        ROS_INFO("Failed to read joint angle\n");
        dRobot.DisconnectRobot();
        return 0;
    }
    for (int i = 0; i < 6;i++){		printf("@CurJang:%d %f \n", i, fValue[i]); }

    // ------------------------------------------------------------------- //

    dRobot.InitializeROS();

    // ------------------------------------------------------------------- //

    ROS_INFO("Press any key to start slave mode.\n");
    getchar();

    cout<<"Launching slave thread...\n";

    // CKim - Creating a thread for running slave mode control.
    // Arguments : (ptr to thread id, NULL means default thread attribute,
    // pointer to function returning void* and taking void* as an argument, pointer to data to pass)
    iret1 = pthread_create( &slaveThread, NULL, DensoRobot::ThrCallback, &dRobot);

    // ------------------------------------------------------------------- //

    // CKim - ros::spin() should be called to publish topics and process
    // subscription callbacks
    while(ros::ok())
    {
        ros::spin();
    }

    // ------------------------------------------------------------------- //

//    cout<<"Are we here??...\n";
    //dRobot.DisconnectRobot();
    cout<<"Sparta!! "<<endl;
    pthread_join(slaveThread, NULL);

    cout<<"Closing...\n";

    return 0;
}

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}


