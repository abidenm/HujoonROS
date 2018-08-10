#include "rt_serial.h"


pthread_mutex_t register_mutex;

rt_serial::rt_serial(){

	device_state = 0;
	rx_error = 0;

    // Map register and data
    for(u_int8_t i = 0; i < 4; i++){
        M[i].pos_goal 	=	(u_int32_t*)(REGISTER + 4 * i);
        M[i].velocity	=	(int32_t*)(REGISTER + 16 + 4 * i);
        M[i].pos		=	(u_int32_t*)(REGISTER + 32 + 4 * i);
        M[i].loadcell 	=	(u_int16_t*)(REGISTER + 48 + 2 * i);
        M[i].current	=	(u_int16_t*)(REGISTER + 56 + 2 * i);
    }
    control_mode	= ((_mode *)(REGISTER + 64));
    LED_pwm			= (u_int16_t*)(REGISTER + 66);
    kp				= (float*)(REGISTER + 68);
    ki				= (float*)(REGISTER + 72);
    kd				= (float*)(REGISTER + 76);
    hall_sensor		= (u_int16_t*)(REGISTER + 80);
}

rt_serial::~rt_serial(){
	if(device_state){
		thread_close();
		serial_close();
	}
}

int rt_serial::serial_open(const char *_device)
{
    m_serialHandle = open(_device, O_RDWR | O_NOCTTY | O_SYNC);
    if (m_serialHandle <0){
		perror(_device);
		return 0;
	}

	memset (&m_oldtio, 0, sizeof m_oldtio);
	memset (&m_newtio, 0, sizeof m_newtio);

    // CKim - Save current serial port settings
    tcgetattr(m_serialHandle,&m_oldtio);	// terminal control get attribute
	bzero(&m_newtio, sizeof(m_newtio)); 	// clear struct for new port settings. disables everything..

	// CKim - Important!! - Set serial port to low latency!!!!!
	struct serial_struct serial;
    ioctl(m_serialHandle, TIOCGSERIAL, &serial);
	serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(m_serialHandle,TIOCSSERIAL, &serial);

	// CKim - Additional Port settings
	m_newtio.c_iflag = IGNPAR;// | IGNBRK;
	m_newtio.c_oflag = 0;
	m_newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	m_newtio.c_lflag = 0;
    m_newtio.c_cc[VMIN] = 0;    // 1
    m_newtio.c_cc[VTIME] = 0;   // 1

    tcsetattr(m_serialHandle,TCSANOW,&m_newtio);

    tcflush(m_serialHandle, TCIFLUSH);
    return m_serialHandle;
}

void rt_serial::serial_close(void){
    tcsetattr(this->m_serialHandle,TCSANOW,&m_oldtio);
    close(this->m_serialHandle);
    ROS_INFO("%s","Closing SerialPort");
}

int rt_serial::InitializeROS()
{
    // CKim - Initialize ROS Subscription
    gui_sub = nh.subscribe("/cath_gui_cmd", 10, &rt_serial::gui_cmd_back, this);
    haptic_sub = nh.subscribe("HapticCmd", 100, &rt_serial::haptic_back,this);
    jstck_sub = nh.subscribe("joy", 100, &rt_serial::joy_back,this);
    catheter_pub = nh.advertise<RobotCatheter::catheterState>("catheter_states", 10);
}

//int rt_serial::rt_sched_set(void){
//	struct sched_param param;

//	param.sched_priority = MY_PRIORITY;
//	if(sched_setscheduler(0, SCHED_FIFO, &param) != 0)	{
//		perror("sched_setscheduler failed");
//		return 0;
//	}
//	return 1;
//}

int rt_serial::thread_open(void){

	int ret;
	device_state = 1;

    for(int i=0; i<4; i++)  {
        *M[i].pos_goal = 0x000C3500;  }

	ret = pthread_create(&(RT_Thread), NULL, RT_Callback, this);
	if(ret){
		device_state = 0;
	}
	return ret;
}

int rt_serial::thread_close(void){
	device_state = 0;
	return pthread_join(RT_Thread,0);
}

int rt_serial::device_homming(void){
	//homming
	char TxBuffer[REG_END];

	TxBuffer[P_HEADER]		= HEADER;
	TxBuffer[P_ID]					= UART_ID;
	TxBuffer[P_RTX]				= RX_FLAG;
	TxBuffer[P_ADDRESS]	= REG_MODE;
	TxBuffer[P_LENGTH]		= 0x02;
	TxBuffer[P_DATA]			= 0x01;
	TxBuffer[P_DATA + 1]	= 0x00;
    write(m_serialHandle, TxBuffer, 7);
	return 1;
}

// Thread function for sending command and read data
void * rt_serial::RT_Callback(void* __this)
{
	rt_serial *_this = (rt_serial *)__this;

    // CKim - Buffer for default communication (goal position, loadcell, motor current..)
    char TxBuffer[REG_END];
	char RxBuffer[REG_END];

    // CKim - Buffer for occasional commands
    char CmdBuffer[REG_END];

    // CKim - RT variables
    long long int time, time_r;
    struct timespec t, t_real;
	struct sched_param param;
	ssize_t ret = 0;

    // CKim - Increase priority (this thread will preempt the kernel functions)
	param.sched_priority = MY_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) != 0)	{
		perror("sched_setscheduler failed");
		_this->device_state = 0;
		return 0;
	}

    // CKim - Set clock
	clock_gettime(CLOCK_REALTIME ,&t); /* start after one second */
	t.tv_nsec += 100*MS;
	tsnorm(&t);

    while((_this->device_state) == 1)
    {
        tcflush(_this->m_serialHandle, TCIFLUSH);

        // --------------------------------
        if(!_this->m_cmdQ.empty())
        {
            CmdBuffer[P_HEADER]		= HEADER;
            CmdBuffer[P_ID]			= UART_ID;
            CmdBuffer[P_RTX]		= _this->m_cmdQ.front().RXTX;
            CmdBuffer[P_ADDRESS]    = _this->m_cmdQ.front().REG;
            CmdBuffer[P_LENGTH]		= _this->m_cmdQ.front().LEN;

            pthread_mutex_lock( &register_mutex);
            memcpy(CmdBuffer + 5, _this->REGISTER + CmdBuffer[P_ADDRESS], CmdBuffer[P_LENGTH]);
            pthread_mutex_unlock( &register_mutex);

            write(_this->m_serialHandle, CmdBuffer, 5 + CmdBuffer[P_LENGTH]);

//            if(CmdBuffer[P_RTX] != RX_FLAG)
//            {
//                // Need a code for reading
//            }
        }
        // --------------------------------


        TxBuffer[P_HEADER]		= HEADER;
        TxBuffer[P_ID]			= UART_ID;
        TxBuffer[P_RTX]			= RTX_FLAG;		// Read and Write
        TxBuffer[P_ADDRESS]     = REG_GOAL_POS;
        TxBuffer[P_LENGTH]		= 16;

        // Write Goal Position and request to read position, load cell, motor current
        pthread_mutex_lock( &register_mutex);
        memcpy(TxBuffer + 5, _this->REGISTER, 16);
        pthread_mutex_unlock( &register_mutex);

        write(_this->m_serialHandle, TxBuffer, 5 + TxBuffer[P_LENGTH]);

        // CKim - Check if loop ran within time
        t.tv_nsec += INTERVAL;
        tsnorm(&t);

        clock_gettime(CLOCK_REALTIME ,&t_real);
        time_r = (long long int)t_real.tv_nsec + (long long int)t_real.tv_sec*NSEC_PER_SEC;
        time  = (long long int)t.tv_nsec + (long long int)t.tv_sec*NSEC_PER_SEC;

        if(time < time_r){
            _this->device_state = 0;
            return 0;
        }

        // CKim - Sleep for remaining time of the loop
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t, NULL);

        // Read position (4 x 4byte), load cell (4 x 2 byte), motor current (4 x 2 byte)
        ret = read(_this->m_serialHandle, RxBuffer, 32);

        if(ret >= 32)
        {
            pthread_mutex_lock( &register_mutex);
            memcpy(_this->REGISTER + 32, RxBuffer, 32);
            pthread_mutex_unlock( &register_mutex);
        }
        else
        {
            _this->rx_error++;
        }

        _this->PublishState();

	}
	_this->device_state = 0;
	return 0;
}

// Copy from variable REGISTER
void rt_serial::get_register(u_int8_t* reg, u_int8_t _offset, u_int8_t _length){
    pthread_mutex_lock( &register_mutex);
    memcpy(reg + _offset, REGISTER + _offset, _length);
    pthread_mutex_unlock( &register_mutex);
}

// Write to the REGISTER
void rt_serial::set_register(u_int8_t* reg, u_int8_t _offset, u_int8_t _length){
    pthread_mutex_lock( &register_mutex);
    memcpy(REGISTER + _offset, reg + _offset, _length);
    pthread_mutex_unlock( &register_mutex);
}

// Directly write to board
void rt_serial::set_register_now(u_int8_t* reg, u_int8_t _offset, u_int8_t _length){
    char TxBuffer[REG_END];


    tcflush(m_serialHandle, TCIFLUSH);

    TxBuffer[P_HEADER]		= HEADER;
    TxBuffer[P_ID]					= UART_ID;
    TxBuffer[P_RTX]				= RX_FLAG;
    TxBuffer[P_ADDRESS]	= _offset;
    TxBuffer[P_LENGTH]		= _length;

    memcpy(TxBuffer + 5, reg + _offset, _length);

    write(m_serialHandle, TxBuffer, 5 + _length);
}

u_int32_t rt_serial::get_err_cnt(void){
    return rx_error;
}

// Read entire register of the board
int rt_serial::device_readstate(void* reg){
    char TxBuffer[REG_END];
    char RxBuffer[REG_END];
    int ret = 0;
    int _i = 0;
    tcflush(m_serialHandle, TCIFLUSH);

    TxBuffer[P_HEADER]		= HEADER;
    TxBuffer[P_ID]					= UART_ID;
    TxBuffer[P_RTX]				= TX_FLAG;
    TxBuffer[P_ADDRESS]	= REG_GOAL_POS;
    TxBuffer[P_LENGTH]		= REG_END;
    write(m_serialHandle, TxBuffer, 5);


    while(ret < REG_END){
        usleep(10000);
        ret = ret + read(m_serialHandle, RxBuffer+ret, REG_END);
        _i++;
        if(_i > 10){
            perror("cannot read data");
            return 0;
        }
    }
    pthread_mutex_lock( &register_mutex);
    memcpy(REGISTER, RxBuffer, REG_END);
    pthread_mutex_unlock( &register_mutex);

    memcpy(reg, RxBuffer, REG_END);

    return 1;
}

void rt_serial::gui_cmd_back(const RobotCatheter::catheterState guicmd)
{
    ROS_INFO("GUI_CMD");
   
    // CKim - Write to the REGISTER. REGISTER is mapped to several pointers
    pthread_mutex_lock( &register_mutex);
    *LED_pwm = guicmd.LightPower;
    pthread_mutex_unlock( &register_mutex);
    
    // CKim - Add to command queue
    cmdData cmd;    
    cmd.RXTX = RX_FLAG;     cmd.REG = REG_LED;      cmd.LEN = 2;
    m_cmdQ.push(cmd);
}

void rt_serial::joy_back(const sensor_msgs::Joy msg)
{
    // CKim - Joystick message msg has ....
    // msg.axes[0] : Left analog stick L(1.0), R(-1.0)    // msg.axes[1] : Left analog stick U(1.0), D(-1.0)
    // msg.axes[3] : Right analog stick L(1.0), R(-1.0)   // msg.axes[4] : Right analog stick U(1.0), D(-1.0)
    // msg.axes[2/5] : Left/Right trigger button pressed (-1.0), released (1.0)

    // After homing Initial position in encoder count is 0x000C3500 = 800000
    // min (0x000000FF = 255. most front) max (0x00186A00 = 1600000)
    // Gear ratio specs. Motor gear head 5.4:1, gear transmission 4:1, screw pitch =  1 mm / turn
    // Encoder resolution 2048/turn
    // Sensor ADC Max is 3.3V = 0xFFF = 4095. Sensor calibrated to 3V at 2 kg.

    // Tendon Driver 1,2,3,4 from left right.

    // CKim - return if thread has not started
//    ROS_INFO("Sparta!!");
    if(device_state == 0)   return;

    // CKim - Calculate target motor count from cmd
    float initCnt = 800000;             float cntPerMtrTurn = 2048;   float MtrTurnPerShaft = 5.4;
    float ShaftTurnPerScrew = -4.0;     float ScrewPitch = 1.0;       float Amp = 16.95;    // = 750000/1/4.0/5.4/2048;

//    // CKim - -1.0 to 1.0 turn cw / ccw 1 revolution
    //float cnt2Screw = cntPerMtrTurn*MtrTurnPerShaft*ShaftTurnPerScrew;
    //        int tgtCnt[4];
//    tgtCnt[0] = initCnt + msg.axes[0]*cnt2Screw;
//    tgtCnt[1] = initCnt + msg.axes[1]*cnt2Screw;
//    tgtCnt[2] = initCnt + msg.axes[3]*cnt2Screw;
//    tgtCnt[3] = initCnt + msg.axes[4]*cnt2Screw;

    // CKim - Push Pull. 0/3 pair, 1/2 pair
    float cnt2Trans = cntPerMtrTurn*MtrTurnPerShaft*ShaftTurnPerScrew*ScrewPitch;
    int tgtCnt[4];
    tgtCnt[0] = initCnt - Amp*msg.axes[3]*cnt2Trans;
    tgtCnt[3] = initCnt + Amp*msg.axes[3]*cnt2Trans;
    tgtCnt[1] = initCnt - Amp*msg.axes[4]*cnt2Trans;
    tgtCnt[2] = initCnt + Amp*msg.axes[4]*cnt2Trans;


    // CKim - Write to the REGISTER. REGISTER is mapped to struct M for convenience
    pthread_mutex_lock( &register_mutex);
    for(int i=0; i<4; i++)  {
        *M[i].pos_goal = (u_int32_t)tgtCnt[i];  }
    pthread_mutex_unlock( &register_mutex);

}


void rt_serial::haptic_back(const SkillMate::HapticCommand cmd)
{

    // After homing Initial position in encoder count is 0x000C3500 = 800000
    // min (0x000000FF = 255. most front) max (0x00186A00 = 160000)
    // Gear ratio specs. Motor gear head 5.4:1, gear transmission 4:1, screw pitch =  1 mm / turn
    // Encoder resolution 2048/turn
    // Sensor ADC Max is 3.3V = 0xFFF = 4095. Sensor calibrated to 3V at 2 kg.

    // Tendon Driver 1,2,3,4 from left right.

//    // CKim - Calculate target motor count from cmd
//    int tgtVal[4];

//    // CKim - Write to the REGISTER
//    pthread_mutex_lock( &register_mutex);
//    for(int i=0; i<4; i++)  {
//        *M[i].pos_goal = (u_int32_t)tgtVal[i];  }
//    pthread_mutex_unlock( &register_mutex);



//    // CKim - Check button press.
//    if(cmd.btn[1] == 1)
//    {
//        m_controlMode = 1;
//        if(m_prevBtn == 0)
//        {
//            m_prevBtn = 1;
//            return;
//        }
//    }
//    else
//    {
//        m_controlMode = 0;
//        m_prevBtn = 0;
//        return;
//    }


//    // CKim LR : array[3] -+ // UD : array[5] +-
//    float LR = 0;   float UD = 0;
//    float rotScl = 7.0;     float ulim = 16.0;
//    float sgn = 1.0;        float val;

//    LR = rotScl*cmd.array[3];       sgn = fabs(LR)/LR;
//    if(fabs(LR) > ulim) {   LR = ulim*sgn;   }

//    UD = rotScl*cmd.array[5];       sgn = fabs(UD)/UD;
//    if(fabs(UD) > ulim) {   UD = ulim*sgn;   }


//    pthread_mutex_lock( &mutex1 );

//    //range : 0xF0(240) ~ 0X3DE(990)
//    // 615-375 < 615 < 615+375
//    val = m_tgtPos[0] - LR;
//    if(val > 250 && val < 980)  {   m_tgtPos[0] = val;     }

//    val = m_tgtPos[1] + UD;
//    if(val > 250 && val < 980)  {   m_tgtPos[1] = val;     }

//    val = m_tgtPos[2] - UD;
//    if(val > 250 && val < 980)  {   m_tgtPos[2] = val;     }

//    val = m_tgtPos[3] + LR;
//    if(val > 250 && val < 980)  {   m_tgtPos[3] = val;     }

//    pthread_mutex_unlock( &mutex1 );
}

void rt_serial::PublishState()
{
    float initCnt = 800000;             float cntPerMtrTurn = 2048;   float MtrTurnPerShaft = 5.4;
    float ShaftTurnPerScrew = -4.0;     float ScrewPitch = 1.0;       float Amp = 16.95;    // = 750000/1/4.0/5.4/2048;

    int cnt[4];

    // CKim - Copy from the REGISTER
    pthread_mutex_lock( &register_mutex);
    for(int i = 0; i < 4; i++){
        cnt[i] = *M[i].pos;
        cState.Tension[i] = *M[i].loadcell;
        cState.Current[i] = *M[i].current;
    }
    pthread_mutex_unlock( &register_mutex);

    for(int i = 0; i < 4; i++){
        cState.Displacement[i] = (cnt[i] - initCnt)/cntPerMtrTurn/MtrTurnPerShaft/ShaftTurnPerScrew;
    }


    catheter_pub.publish(cState);
}


//set overflow of time struct
inline void tsnorm(struct timespec *ts){
	while (ts->tv_nsec >= NSEC_PER_SEC)	{
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}

