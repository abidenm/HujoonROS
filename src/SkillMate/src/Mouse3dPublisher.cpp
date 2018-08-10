// -------------------------------------------------------- //
// CKim - This ROS node reads 3D Space Mouse command and
// publishes it
// -------------------------------------------------------- //

//--------------------------------ggw-----------------------
// linux device driver and SDK source http://spacenav.sourceforge.net/
// device driver : spacenav daemon (read README file and install device driver)
// SDK : copy spacenav library
//     remove X11 parts

// capy simple.c :
//  change name 3dMouse.cpp
//    remove X11 parts
//    add ROS parts
//--------------------------------ggw-----------------------


#include "ros/ros.h"
#include <SkillMate/Mouse3dCommand.h>


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <spnav_ggw.h>

void sig(int s)
{
    //spacenav library
    spnav_close();
    ROS_INFO("%s","close mouse_3d control.\n");
    exit(0);
}

int main(int argc, char **argv)
{

    spnav_event sev;

    signal(SIGINT, sig);

    //open 3d mouse device
    //spacenav library
    if(spnav_open()==-1) {
        ROS_INFO("%s", "failed to connect to the space navigator daemon\n");
        return 1;
    }


    ros::init(argc, argv, "mouse_3d");
    ros::NodeHandle nh;
    ros::Publisher mouse_pub = nh.advertise<SkillMate::Mouse3dCommand>("mouse_3d", 100);
    ros::Rate loop_rate(30);
    ROS_INFO("%s","Starting 3D_Mouse control.\n");

    while(ros::ok()) {
       SkillMate::Mouse3dCommand msg;

        //spacenav library
        //if have new data, publish message
        if(spnav_poll_event(&sev))
        {
            /*
            sev.type :
            SPNAV_EVENT_ANY or
            SPNAV_EVENT_MOTION(detect motion) or
            SPNAV_EVENT_BUTTON(detect button press or release) or
            */
            msg.type = sev.type;

            msg.x = sev.motion.x;
            msg.y = sev.motion.y;
            msg.z = sev.motion.z;

            msg.rx = sev.motion.rx;
            msg.ry = sev.motion.ry;
            msg.rz = sev.motion.rz;

            /*
            sev.button.press :
            FULSE : release
            TRUE : press
            */
            msg.press = sev.button.press;

            /*
            sev.button.bnum :
            0 or 1
            */
            msg.button = sev.button.bnum;
            //publish mouse message
            mouse_pub.publish(msg);
            //ROS_INFO("msg_pub");
        }
        else
            loop_rate.sleep();
    }

    //spacenav library
    //close device
    spnav_close();
    ROS_INFO("%s","close 3D_Mouse control.\n");
    return 0;
}


