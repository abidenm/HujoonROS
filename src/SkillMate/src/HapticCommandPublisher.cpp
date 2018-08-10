// -------------------------------------------------------- //
// CKim - This ROS node reads HapticDevice command from
// network and publishes it
// -------------------------------------------------------- //

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string>
#include <time.h>
#include "ros/ros.h" // ROS 기본 헤더 파일
#include <SkillMate/HapticCommand.h>        // CKim - Automatically generated message Header

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char* argv[])
{
	int sock;
	struct sockaddr_in server_addr;

    if(argc<3)
	{
        printf("Usage : %s <IP> <port> \n", argv[3]);
		exit(1);
	 }

    // Client Socket
	sock = socket(PF_INET, SOCK_STREAM, 0);

	//=======
    ros::init(argc, argv, "HapticCmdPublisher");    //노드 초기화
    ros::NodeHandle nh; //노드 핸들 선언

    ros::Publisher haptic_pub =	nh.advertise<SkillMate::HapticCommand>("HapticCmd", 100);
    ROS_INFO("Starting Haptic Node");
    //====

	// Connect to address
	memset(&server_addr, 0, sizeof(server_addr));//서버 주소 초기화
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(argv[1]);
    server_addr.sin_port = htons(atoi(argv[2]));

    ROS_INFO("Connecting to Server!\n");
	if(connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1)
	{
		printf("connect() error\n");
		close(sock);
		exit(1);
	}
    ROS_INFO("Connected to Server!\n");

	//ros::Rate loop_rate(10);
	//ros::WallTime currTime = ros::WallTime::now();
	while(ros::ok())
	{
		// ----------------------------------------------------------- //
		// CKim - This code is using no protocol. Just 6 doubles
		// ----------------------------------------------------------- //
		int leng, real_recv_len, real_recv_byte;

        leng = 6*sizeof(double) + 2*sizeof(int);
		char* str2 = new char[leng]; //길이 만큼 배열 동적할당

		if (str2 == (char *) 0) {
			printf("memory error!\n");
			exit(1);
		}

		memset(str2, 0, leng);
		real_recv_len = 0;
		real_recv_byte = 0;

		//받는 방법 2 : 받는 바이트 확인하며 받기
		while (real_recv_len < leng) {
			real_recv_byte = read(sock, &str2[real_recv_len],
					leng - real_recv_len);
			real_recv_len += real_recv_byte;
		}

		double* val = (double*) str2;
        int* btn = (int*) (str2+6*sizeof(double));

        SkillMate::HapticCommand msg;

		for(int i=0; i<3; i++)	{
			msg.array[i] = val[i];			}
		for(int i=3; i<6; i++)	{
            msg.array[i] = val[i];  }//*2000.0;			}
		msg.btn[0] = btn[0];
		msg.btn[1] = btn[1];

		// ----------------------------------------------------------- //

		// 메시지를 퍼블리시 한다.
		haptic_pub.publish(msg);

		//loop_rate.sleep();
	}
	close(sock);
	return 0;
}

