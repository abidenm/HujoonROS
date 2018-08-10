// --------------------------------------------------------------------- //
// CKim - ROS node for communicating with the motor controller
// of the Robot Catheter
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

#include "rt_serial.h"

// CKim - ROS include
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv)
{
    // CKim - Lock memory fo rt-process
    char *buffer;            int page_size;

     if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        perror("mlockall failed"); exit(-2);
     } /* Pre-fault our stack */

    mallopt (M_TRIM_THRESHOLD, -1);      // Turn off malloc trimming.
    mallopt (M_MMAP_MAX, 0);                // Turn off mmap usage.

    page_size = sysconf(_SC_PAGESIZE);
    buffer = (char *)malloc(POOLSIZE);

    for (int i=0; i < POOLSIZE; i+=page_size)
        buffer[i] = 0;

    free(buffer);

    ROS_INFO("Memory Locked");

    // CKim - Init ROS
    ros::init(argc, argv, "Catheter");

    rt_serial m_rtSerial;
    m_rtSerial.InitializeROS();

    ROS_INFO("ROS Initialized. Opening Serial Port");

    if(!m_rtSerial.serial_open("/dev/ttyUSB0"))  {
        ROS_INFO("Failed to open connection to catheter");
        return 0;
    }

    // ------------------------------------------------------------------- //
    ROS_INFO("Port Opened. Press any key to home the Robot.");
    getchar();
    m_rtSerial.device_homming();

    ros::Duration(5).sleep();

    ROS_INFO("Homing Complete. Press any key to launch RS485 communication thread.");
    getchar();


    // CKim - Creating a thread for running slave mode control.
    // Arguments : (ptr to thread id, NULL means default thread attribute,
    // pointer to function returning void* and taking void* as an argument, pointer to data to pass)
    //iret1 = pthread_create( &catheterThread, NULL, CatheterDriver::ThrCallback, &Driver);
    m_rtSerial.thread_open();
    ROS_INFO("Thread Started. Entering Control Loop.\n");

    // ------------------------------------------------------------------- //

    // CKim - ros::spin() should be called to publish topics and process
    // subscription callbacks
    while(ros::ok())
    {
        ros::spin();
    }

    // ------------------------------------------------------------------- //

    //pthread_join(catheterThread,0);
    m_rtSerial.thread_close();

    //Driver.Disconnect();
    m_rtSerial.serial_close();
    ROS_INFO("Exiting Catheter Driver node");

    return 0;
}

