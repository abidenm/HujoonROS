#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <string>
#include <time.h>

#include "ros/ros.h" // ROS 기본 헤더 파일
#include <SkillMate/HapticCommand.h>            // CKim - Automatically generated message Header
#include <sensor_msgs/Joy.h>

using namespace std;

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
float pData[8];

class joy2Hap
{
public:
    joy2Hap();

private:
    void joy_back(const sensor_msgs::Joy msg);

    ros::NodeHandle nh; //노드 핸들 선언
    ros::Publisher haptic_pub;
    ros::Subscriber jstck_sub;
};

joy2Hap::joy2Hap()
{
    // CKim - Initialize publisher, subscriber
    haptic_pub = nh.advertise<SkillMate::HapticCommand>("HapticCmd", 100);
    jstck_sub = nh.subscribe("joy", 100, &joy2Hap::joy_back,this);
}


int main(int argc, char* argv[])
{
    // CKim - Initialize the publisher/subscriber
    ros::init(argc, argv, "Joy2Hap");    //노드 초기화

    // CKim - Constructor of the class handles all the initialization
    joy2Hap jH;

    ROS_INFO("Starting the Node!\n");

    ros::spin();

	return 0;
}

void joy2Hap::joy_back(const sensor_msgs::Joy msg)
{
    //ROS_INFO("Sparta!!");
    SkillMate::HapticCommand hapMsg;

    // CKim - XYZ position to joystick input
    hapMsg.array[0] = -msg.axes[0];    hapMsg.array[2] = msg.axes[1];
    if(msg.axes[2] < msg.axes[5])   {   hapMsg.array[1] = 1.0 - msg.axes[2];    }
    else                            {   hapMsg.array[1] = -1.0 + msg.axes[5];    }

    // CKim - XYZ rotation to joystick input
    hapMsg.array[3] = msg.axes[4];       hapMsg.array[4] = msg.axes[3];
    hapMsg.array[5] = 0;

    hapMsg.btn[0] = msg.buttons[0];

    haptic_pub.publish(hapMsg);
}
