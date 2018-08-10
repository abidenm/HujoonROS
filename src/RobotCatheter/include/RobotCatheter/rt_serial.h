#ifndef RT_SERIAL_H
#define RT_SERIAL_H

// ----------------------------------------------------
// GGW & CKim - Class encapsulating serial communication
// to the motor controller board developed by
// GGW for Dr. Hujoon Robot Catheter
// ----------------------------------------------------

/* rt define */
#define NS  	(1)
#define US  	(1000 * NS)
#define MS  	(1000 * US)
#define SEC 	(1000 * MS)

#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50 as the priority of kernel tasklets and interrupt handler by default */
#define POOLSIZE (1*1024*1024) /* The maximum stack size which is guaranteed safe to access without faulting */
//#define POOLSIZE (1*512*512) /* The maximum stack size which is guaranteed safe to access without faulting */
#define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */
#define N_EVER 100
#define INTERVAL (10 * MS)

#define MODEMDEVICE "/dev/ttyUSB0"
#define BAUDRATE B921600

#define HOME_POS 0x000C3500

// After homing Initial position in encoder count is 0x000C3500 = 800000
// min (0x000000FF = 255. most front) max (0x00186A00 = 160000)
// Gear ratio specs. Motor gear head 5.4:1, gear transmission 4:1, screw pitch =  1 mm / turn
// Encoder resolution 2048/turn
// Sensor ADC Max is 3.3V = 0xFFF = 4095. Sensor calibrated to 3V at 2 kg. 

// Tendon Driver 1,2,3,4 from left right.
// LED 0 to 4999

/*
* communication packet
* Byte 0 : 0x55						// HEADER
* Byte 1 : ID						// Board id
* Byte 2 : RX TX flag				// RX_FLAG 0X00, TX_FLAG 0X01, RTX_FLAG 0X02
* Byte 3 : register address         // Start Adress of the register to read/write
* Byte 4 : length					// Length of data to write from the start address
* Byte 5 ~ end : rx data			// Data to be written to the board. Length of the rx data = Byte 4
*
*RX_FLAG 0X00 : (write data to board)
*TX_FLAG 0X01 : (read data from board)
*RTX_FLAG 0X02 : (read and write)
*/

// For example, set the goal position of 4 motors controlled by Board 1, packets will be
// Byte 0 : 0x55	// Header
// Byte 1 : 0x01	// Board 1
// Byte 2 : 0x00	// RX_FLAG
// Byte 3 : 0x00	// Goal position register starts from 0.	Register for goal Position0/1/2/3 = 0/4/8/12
// Byte 4 : 0x10	// 16 Bytes. One goal position = 4 byte unsigned int
// Byte 5-20 : Goal Position Data

// For example, to read current position of 4 motors controlled by Board 1, packets will be
// Byte 0 : 0x55	// Header
// Byte 1 : 0x01	// Board 1
// Byte 2 : 0x01	// TX_FLAG
// Byte 3 : 0x20	// Current position register starts from 32.Register for current Position0/1/2/3 = 32/36/40/44
// Byte 4 : 0x10	// 16 Bytes. One position = 4 byte unsigned int
// And wait for incoming packet

#define P_HEADER		0x00
#define P_ID			0x01
#define P_RTX			0x02
#define P_ADDRESS       0x03
#define P_LENGTH		0x04
#define P_DATA			0x05

#define HEADER			0X55
#define UART_ID			0X01

#define RX_FLAG			0x00
#define TX_FLAG			0x01
#define RTX_FLAG		0x02


/*
***** register address *****
*  0~ : m0 goal position(4bytes, u_int32_t)
*  4~ : m1 goal position(4bytes, u_int32_t)
*  8~	: m2 goal position(4bytes, u_int32_t)
* 12~ : m3 goal position(4bytes, u_int32_t)
* 16~ : m0 velocity(4bytes, u_int32_t)
* 20~ : m1 velocity(4bytes, u_int32_t)
* 24~ : m2 velocity(4bytes, u_int32_t)
* 28~ : m3 velocity(4bytes, u_int32_t)
* 32~ : m0 position(4bytes, u_int32_t)
* 36~ : m1 position(4bytes, u_int32_t)
* 40~ : m2 position(4bytes, u_int32_t)
* 44~ : m3 position(4bytes, u_int32_t)
* 48~ : m0 load cell(2bytes, u_int16_t)
* 50~ : m1 load cell(2bytes, u_int16_t)
* 52~ : m2 load cell(2bytes, u_int16_t)
* 54~ : m3 load cell(2bytes, u_int16_t)
* 56~ : m0 current(2bytes, u_int16_t)
* 58~ : m1 current(2bytes, u_int16_t)
* 60~ : m2 current(2bytes, u_int16_t)
* 62~ : m3 current(2bytes, u_int16_t)
* 64~ : control mode(2bytes, u_int16_t)	// 0: PID Control	1: Moving to limit switch for homing 2: Returning to home position
* 66~ : LED PWM(2bytes, u_int16_t)
* 68~ : Kp gain(4bytes, float)
* 72~ : Ki gain(4bytes, float)
* 76~ : Kd gain(4bytes, float)
* 80~ : hall sensor(2bytes, u_int16_t)	// bit 0-7 = hall sensor 0-7 on off
* 82~ : count1(2bytes, u_int16_t)
* 84~ : count2(2bytes, u_int16_t)
* 86~ : count3(2bytes, u_int16_t)
* 88~ : count4(2bytes, u_int16_t)
*/

#define REG_GOAL_POS			0
#define REG_VELOCITY			16
#define REG_POS					32
#define REG_ROAD_CELL			48
#define REG_CURRENT				56
#define REG_MODE				64
#define REG_LED					66
#define REG_Kp					70
#define REG_Ki					74
#define REG_Kd					78
#define REG_HALL_SEN			80
#define REG_COUNT1				82
#define REG_COUNT2				84
#define REG_COUNT3				86
#define REG_COUNT4				88

#define REG_END					90

// CKim - ROS include
#include <ros/ros.h>

// CKim - Headers for the published / subscribed message.
#include <SkillMate/Mouse3dCommand.h>
#include <SkillMate/HapticCommand.h>
#include <RobotCatheter/catheterState.h>
#include <sensor_msgs/Joy.h>

// CKim - Other Headers
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>

#include <signal.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <linux/serial.h>

#include <queue>

// CKim - Structs and Enums
enum  motor_state{
	normal = 0,
	hall_l,
	hall_h
};

typedef enum  _mode{
	control_mode_position = 0,
	control_mode_homing
} ctr_mode;

typedef struct {
	//enum  motor_state state;
	u_int16_t enc_h;		//encorder high value
	u_int16_t enc_l;		//encorder low value
	u_int32_t *pos;		//encorder position address(REGEISTER 16 ~ 31)(high + low value)
	u_int32_t *pos_goal; //encorder goal position address(REGEISTER 0 ~ 15)
	u_int32_t pos_old;	//encorder old position

	int32_t pos_err;	//encorder pos_goal - pos
	int32_t pos_esum;	//pos error sum(intagration value)
	int32_t *velocity; 	//pos - pos_old address(REGEISTER 32 ~ 39)

	int32_t output;		//pwm output

    u_int16_t *loadcell; //road cell value
	u_int16_t *current;	//current value
} motorData;

typedef struct {
    char RXTX; // RX_FLAG 0X00 (writes to board), TX_FLAG 0X01 (read from board), RTX_FLAG 0X02
    char REG;   // Start Adress of the register to read/write
    char LEN;   // Length of data to write from the start address
} cmdData;


class rt_serial
{

private:
	struct termios m_oldtio, m_newtio;		// Serial setting
    pthread_t RT_Thread;					// Thread handle
	u_int8_t REGISTER[REG_END];				// Array storing entire register
    u_int32_t rx_error;						// count error
    int m_serialHandle;
    int device_state;						// 1: rt thread running

	ctr_mode* control_mode;
	motorData M[4];
	u_int16_t *LED_pwm;
	u_int16_t * hall_sensor;

	float *kp;
	float *ki;
	float *kd;

    // CKim - Internal functions
    void get_register(u_int8_t* reg, u_int8_t _offset, u_int8_t _length);
    void set_register(u_int8_t* reg, u_int8_t _offset, u_int8_t _length);
    void set_register_now(u_int8_t* reg, u_int8_t _offset, u_int8_t _length);
    u_int32_t get_err_cnt(void);
    int device_readstate(void* reg);
    static void * RT_Callback(void* x);

    // CKim - Callback for subscribed ROS message
    void gui_cmd_back(const RobotCatheter::catheterState cmd);
    void haptic_back(const SkillMate::HapticCommand msg);
    void joy_back(const sensor_msgs::Joy msg);
    void PublishState();

    // CKim - ROS variables
    ros::NodeHandle nh;
    ros::Subscriber gui_sub;
    ros::Subscriber haptic_sub;
    ros::Subscriber jstck_sub;
    RobotCatheter::catheterState cState;
    ros::Publisher catheter_pub;

    std::queue<cmdData> m_cmdQ;
    
public:
	rt_serial();
	~rt_serial();

//	int rt_init(void);
//	int rt_sched_set(void);

    // CKim - RS485 port open and close ...
	int serial_open(const char *_device);
	void serial_close(void);

    // CKim - Initialize ROS publishing and subscription
    int InitializeROS();

	int thread_open(void);		// Start motion control thread
	int thread_close(void);
	int device_homming(void);
};


inline void tsnorm(struct timespec *ts);

#endif // RT_SERIAL_H
