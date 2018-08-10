/*
 * This file is part of the 쨉OS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
// v04_cpLED_ADCmf (v04에 v04_LED에 ADC MF 부분 카피)
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f4xx.h"

// ----------------------------------------------------------------------------
//
// Semihosting STM32F4 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


#define limiter(value, cut)	(value > cut) ? cut : ((value < -cut) ? -cut : value);

/* common define */
#define ON 	1
#define OFF 0

/* usart define */
#define BAUDRATE  921600
#define HEADER 0x55					//UART HEADER
#define ID 0x01						//UART ID
#define RX_FLAG 	0X00			//UART RX or TX flag
#define TX_FLAG	 	0X01			//UART RX or TX flag
#define RTX_FLAG 	0X02			//UART RX or TX flag


#define REGISTER_END 90				//REGISTER area
#define UART_TIMEOUT 10 			//UART TIME OUT (x 100US)

/* motor define*/

#define HOME_POS 0x000C3500
#define POS_INIT HOME_POS			//MOTOR ON INIT POSITION

#define ERR_SUM_LIMIT 3000			//PID I VELUE LIMIT
#define POSITION_SENSITIVITY 100	//POSITION_SENSITIVITY
#define PWM_LIMIT 995				//PWM LIMIT(0 ~ 1000)
#define DEFAULT_KP  1.0f			//DEFAULT KP GAIN
#define DEFAULT_KI  0.0001f			//DEFAULT KI GAIN
#define DEFAULT_KD  0.00001f		//DEFAULT KD GAIN


/* register define */
//MOTOR PWM REGISTER
#define M0_PWM TIM1->CCR1
#define M1_PWM TIM1->CCR2
#define M2_PWM TIM1->CCR3
#define M3_PWM TIM1->CCR4

//MOTOR DIRECTION SET
#define M0_OFF GPIOB->BSRR = GPIO_BSRR_BR_12|GPIO_BSRR_BR_13
#define M0_CCW GPIOB->BSRR = GPIO_BSRR_BS_12|GPIO_BSRR_BR_13
#define M0_CW  GPIOB->BSRR = GPIO_BSRR_BR_12|GPIO_BSRR_BS_13

#define M1_OFF GPIOB->BSRR = GPIO_BSRR_BR_14|GPIO_BSRR_BR_15
#define M1_CCW GPIOB->BSRR = GPIO_BSRR_BS_14|GPIO_BSRR_BR_15
#define M1_CW  GPIOB->BSRR = GPIO_BSRR_BR_14|GPIO_BSRR_BS_15

#define M2_OFF GPIOC->BSRR = GPIO_BSRR_BR_6|GPIO_BSRR_BR_7
#define M2_CCW GPIOC->BSRR = GPIO_BSRR_BS_6|GPIO_BSRR_BR_7
#define M2_CW  GPIOC->BSRR = GPIO_BSRR_BR_6|GPIO_BSRR_BS_7

#define M3_OFF GPIOC->BSRR = GPIO_BSRR_BR_8|GPIO_BSRR_BR_9
#define M3_CCW GPIOC->BSRR = GPIO_BSRR_BS_8|GPIO_BSRR_BR_9
#define M3_CW  GPIOC->BSRR = GPIO_BSRR_BR_8|GPIO_BSRR_BS_9

//HALL SENSOR VELUE GET
#define HALL_0_0 (uint8_t)(!(GPIOC->IDR & GPIO_IDR_IDR_15))//J22
#define HALL_0_1 (uint8_t)(!(GPIOC->IDR & GPIO_IDR_IDR_14))//J21
#define HALL_1_0 (uint8_t)(!(GPIOC->IDR & GPIO_IDR_IDR_13))//J20
#define HALL_1_1 (uint8_t)(!(GPIOC->IDR & GPIO_IDR_IDR_12))//J19
#define HALL_2_0 (uint8_t)(!(GPIOC->IDR & GPIO_IDR_IDR_11))//J18
#define HALL_2_1 (uint8_t)(!(GPIOC->IDR & GPIO_IDR_IDR_10))//J17
#define HALL_3_0 (uint8_t)(!(GPIOA->IDR & GPIO_IDR_IDR_7))//J16
#define HALL_3_1 (uint8_t)(!(GPIOA->IDR & GPIO_IDR_IDR_6))//J15


//LED PWM define
#define LED_PWM_CCR		TIM10->CCR1

//ADC Moving Filter define
#define VAL_EMAF_K			0.1f
#define VAL_EMAF_K_INV		0.9f


//uart state value
enum packet{
	P_HEADER = 0,
	P_ID,
	P_RTX,
	P_ADDRESS,
	P_LENGTH,
	P_TXDATA,
	P_RXDATA,
	P_RTXDATA

} uart_state= P_HEADER;



enum  Mode{
	controll_mode_position = 0,
	controll_mode_homing_flont,
	controll_mode_homing_back
}  *control_mode;


/*
 ***** register address *****
 *  0~ : m0 goal position(4bytes)
 *  4~ : m1 goal position(4bytes)
 *  8~ : m2 goal position(4bytes)
 * 12~ : m3 goal position(4bytes)
 * 16~ : m0 velocity(4bytes)
 * 20~ : m1 velocity(4bytes)
 * 24~ : m2 velocity(4bytes)
 * 28~ : m3 velocity(4bytes)
 * 32~ : m0 position(4bytes)
 * 36~ : m1 position(4bytes)
 * 40~ : m2 position(4bytes)
 * 44~ : m3 position(4bytes)
 * 48~ : m0 load cell(2bytes)
 * 50~ : m1 load cell(2bytes)
 * 52~ : m2 load cell(2bytes)
 * 54~ : m3 load cell(2bytes)
 * 56~ : m0 current(2bytes)
 * 58~ : m1 current(2bytes)
 * 60~ : m2 current(2bytes)
 * 62~ : m3 current(2bytes)
 * 64~ : control mode(2bytes)
 * 66~ : LED PWM(2bytes)
 * 68~ : Kp gain(4bytes)
 * 72~ : Ki gain(4bytes)
 * 76~ : Kd gain(4bytes)
 * 80~ : hall sensor(2bytes)
 * 82~ : count1(2bytes)
 * 84~ : count2(2bytes)
 * 86~ : count3(2bytes)
 * 88~ : count4(2bytes)
 */
uint8_t REGISTER[REGISTER_END];




/*
 * MO ~ M4 motor condition structure
 */

struct {
	uint16_t enc_h;		//encoder high value
	uint16_t enc_l;		//encoder low value
	uint32_t *pos;		//encoder position address(REGEISTER 16 ~ 31)(high + low value)
	uint32_t *pos_goal; //encoder goal position address(REGEISTER 0 ~ 15)
	uint32_t pos_old;	//encoder old position

	int32_t pos_err;	//encoder pos_goal - pos
	int32_t pos_esum;	//pos error sum(intagration value)
	int32_t *velocity; 	//pos - pos_old address(REGEISTER 32 ~ 39)

	int32_t output;		//pwm output
	//int32_t *p_output;	//debugging for check pwm_output(18.08.02)

	uint16_t *loadcell; //load cell value
	uint16_t *current;	//current value

	int16_t(*get_enc)(void);//get encoder value
	void(*set_motor)(int32_t);
}	M[4];

uint16_t * adc[8];

uint16_t *LED_pwm;
uint16_t * hall_sensor;

float *kp;
float *ki;
float *kd;

uint16_t *debug_cnt1;
uint16_t *debug_cnt2;
uint16_t *debug_cnt3;
uint16_t *debug_cnt4;

uint8_t adc_flag = OFF;
uint8_t uart_count = 0;


//ADC EMAF 자료구조
uint16_t adcAvr_cur[8] = {0,};
uint16_t adcAvr_pre[8] = {0,};
uint32_t adcEMA_cur[8] = {0.0, };
uint32_t adcEMA_pre[8] = {0.0, };



void interrupt_init(void);
void GPIO_init(void);
void enc_init(void);
void PWM_init(void);
void timer_interrupt_init(void);
void USART_init(void);
void ADC_init(void);
void LED_init(void);

void USART_transmit(uint8_t *data, uint16_t length);
inline void USART_TX_start(uint8_t *_data);
inline void USART_TX_data(uint8_t *_data);
inline void USART_TX_end(void);


int16_t get_enc_M0(void);
int16_t get_enc_M1(void);
int16_t get_enc_M2(void);
int16_t get_enc_M3(void);

void set_motor0(int32_t pwm);
void set_motor1(int32_t pwm);
void set_motor2(int32_t pwm);
void set_motor3(int32_t pwm);

/*
 * 	M0~M3 encoder low 16bit overflow interrupt vector, high 16bit update
 */
void TIM2_IRQHandler(){
	TIM2->SR &= ~TIM_SR_UIF;
	if(TIM2->CNT > 0x7fff){
		M[0].enc_h++;
	}
	else{
		M[0].enc_h--;
	}
}

void TIM3_IRQHandler(){
	TIM3->SR &= ~TIM_SR_UIF;
	if(TIM3->CNT < 0x7fff){
		M[1].enc_h++;
	}
	else{
		M[1].enc_h--;
	}
}

void TIM4_IRQHandler(){
	TIM4->SR &= ~TIM_SR_UIF;
	if(TIM4->CNT > 0x7fff){
		M[2].enc_h++;
	}
	else{
		M[2].enc_h--;
	}
}

void TIM5_IRQHandler(){
	TIM5->SR &= ~TIM_SR_UIF;
	if(TIM5->CNT < 0x7fff){
		M[3].enc_h++;
	}
	else{
		M[3].enc_h--;
	}
}

// v04_cpLED_ADCmf (v04에 v04_LED에 ADC MF 부분 카피)

/*
 * timer interrupt vector
 * 100us, 10000Hz
 */
void TIM1_BRK_TIM9_IRQHandler(){

	if ((TIM9->SR & TIM_SR_UIF) && (TIM9->DIER & TIM_DIER_UIE)){

		//GPIOB->BSRR = GPIO_BSRR_BS_8;

		//timer overflow interrupt flag clear
		TIM9->SR = (uint16_t)~TIM_SR_UIF;

		*hall_sensor = (uint16_t)((HALL_3_1<<7)|(HALL_3_0<<6)|(HALL_2_1<<5)|(HALL_2_0<<4)|(HALL_1_1<<3)|(HALL_1_0<<2)|(HALL_0_1<<1)|(HALL_0_0));

		switch(*control_mode){
		case controll_mode_position :
			break;
		case controll_mode_homing_flont :
			if(*hall_sensor == 0x0055){
				*control_mode = controll_mode_homing_back;

				TIM2->CNT = 0xff00;
				TIM3->CNT = 0x0100;
				TIM4->CNT = 0xff00;
				TIM5->CNT = 0x0100;
				for(uint8_t _i = 0;_i < 4;_i++){
					M[_i].enc_h = 0;
					*M[_i].pos_goal = HOME_POS;
				}
				break;
			}

			for(uint8_t _i = 0;_i < 4;_i++){
				if(!((*hall_sensor >> ((_i * 2))) & 0x01)){
					*M[_i].pos_goal -= 20;
				}
			}
			break;
		case controll_mode_homing_back :
			if( (*M[0].pos <= (HOME_POS + POSITION_SENSITIVITY))&(*M[0].pos >= (HOME_POS - POSITION_SENSITIVITY))&
				(*M[1].pos <= (HOME_POS + POSITION_SENSITIVITY))&(*M[1].pos >= (HOME_POS - POSITION_SENSITIVITY))&
				(*M[2].pos <= (HOME_POS + POSITION_SENSITIVITY))&(*M[2].pos >= (HOME_POS - POSITION_SENSITIVITY))&
				(*M[3].pos <= (HOME_POS + POSITION_SENSITIVITY))&(*M[3].pos >= (HOME_POS - POSITION_SENSITIVITY))){
			*control_mode = controll_mode_position;
			}
			break;
		default :
			*control_mode = controll_mode_position;
			break;
		}



		//M0 ~ M3 MOTOR PID control
		for(uint8_t _i = 0;_i < 4;_i++){

			M[_i].pos_old 	= *M[_i].pos;

			// get encoder low 16bit value
			M[_i].enc_l = M[_i].get_enc();

			//encoder high 16bit + low 16bit
			*M[_i].pos 		= ((uint32_t)(M[_i].enc_h) << 16) + (uint32_t)M[_i].enc_l;

			//update velocity, error, error sum
			*M[_i].velocity	= *M[_i].pos - M[_i].pos_old;
			M[_i].pos_err 	= *M[_i].pos - *M[_i].pos_goal;
			M[_i].pos_esum +=  M[_i].pos_err;

			M[_i].pos_esum = limiter(M[_i].pos_esum, ERR_SUM_LIMIT);

			//position sensitivity check
			if(POSITION_SENSITIVITY < abs(M[_i].pos_err)){
				M[_i].output = (int32_t)((*kp) * (float)M[_i].pos_err + (*ki) * (float)M[_i].pos_esum + (*kd) * (float)(*M[_i].velocity));
				M[_i].output = limiter(M[_i].output, PWM_LIMIT);
			}
			else{
				M[_i].output = 0;
			}
			//for debugging (check pwm_output)(18.08.02)
			//*M[_i].p_output = M[_i].output;

			//check hall sensor and set pwm, dir
			M[_i].set_motor(M[_i].output);
		}

		//check adc end
		if(!adc_flag){
			adc_flag = ON;
			ADC1->CR2 |= ADC_CR2_SWSTART;
		}

		//check uart time out
		if(uart_state != P_HEADER){
			uart_count++;
			if(UART_TIMEOUT < uart_count){
				USART_TX_end();
				uart_state = P_HEADER;
				uart_count = 0;
			}
		}

		//LED PWM 제어
		LED_PWM_CCR = *LED_pwm;

		//GPIOB->BSRR = GPIO_BSRR_BR_8;
	}
}


/*
 * communication packet
 * 0 : HEADER0x55
 * 1 : ID
 * 2 : RX TX flag
 * 3 : register address
 * 4 : length
 * 5 ~ 4 + length : rx data
 */

/*
 * uart rx interrupt
 */
void USART2_IRQHandler()
{

	static uint8_t rx_i = 0;
	static uint8_t rx_buf[64];
	static uint8_t rtx_flag;
	static uint8_t address;
	static uint8_t length;
	static uint8_t tx_cnt = 0;

	uint8_t rxdata;
	//check rx interrupt
	if(USART2->SR & (USART_SR_RXNE|USART_SR_ORE)){
		//get rx data
		rxdata = USART2->DR;
		//check communication packet
		switch(uart_state){
		case P_HEADER://check header
			if(rxdata == HEADER){
				uart_state= P_ID;
				uart_count = 0;
				(*debug_cnt1)++;
			}
			break;
		case P_ID://check id
			if(rxdata == ID){
				uart_state= P_RTX;
			}
			else{//id error
				uart_state= P_HEADER;
			}
			break;
		case P_RTX://check rx tx flag
			switch(rxdata){
			case RX_FLAG:
			case TX_FLAG://rx, tx flag setting
			case RTX_FLAG:
				rtx_flag = rxdata;
				uart_state= P_ADDRESS;
				break;
			default://rx, tx flag error
				uart_state= P_HEADER;
				break;
			}
			break;
		case P_ADDRESS://register address setting
			address = rxdata;
			if(REGISTER_END < address){//check register address range
				uart_state= P_HEADER;
			}
			else{
				uart_state= P_LENGTH;
			}
			break;
		case P_LENGTH://check length
			length = rxdata;
			if(REGISTER_END < (address + length)){// check register address +length range
				uart_state= P_HEADER;
				break;
			}
			switch(rtx_flag){
			case TX_FLAG:
				/*send data*/
				tx_cnt = 0;
				USART_TX_start(REGISTER + address);
				tx_cnt ++;
				uart_state= P_TXDATA;

				break;
			case RX_FLAG:
				uart_state= P_RXDATA;
				rx_i = 0;
				break;
			case RTX_FLAG:
				uart_state= P_RTXDATA;
				rx_i = 0;
				break;
			default:
				uart_state= P_HEADER;
			}
			break;

		case P_RXDATA:
			/*receive data*/
			if(rx_i < length - 1){//get rx data
				rx_buf[rx_i++] = rxdata;
			}
			else{// set rx data to REGISTER
				rx_buf[rx_i] = rxdata;
				for(uint8_t _i = 0;_i < length;_i++){
					*(REGISTER + address + _i) = rx_buf[_i];
				}
				uart_state= P_HEADER;
			}
			break;

		case P_RTXDATA:
			/*receive data*/
			if(rx_i < length - 1){//get rx data
				rx_buf[rx_i++] = rxdata;
			}
			else{// set rx data to REGISTER
				rx_buf[rx_i] = rxdata;
				for(uint8_t _i = 0;_i < length;_i++){
					*(REGISTER + address + _i) = rx_buf[_i];
				}
				address = 32;
				length = 32;
				tx_cnt = 0;
				USART_TX_start(REGISTER + address);
				tx_cnt ++;
			}
			break;

		default :
			uart_state= P_HEADER;
			break;
		}
	}
	else if(USART2->SR & USART_SR_TC){
		USART_TX_end();
		(*debug_cnt2)++;
		uart_state = P_HEADER;
	}
	else if(USART2->SR & USART_SR_TXE){
		if((uart_state == P_TXDATA)||(uart_state == P_RTXDATA)){
			if(tx_cnt < length){
				USART_TX_data(REGISTER + address + tx_cnt);
				tx_cnt++;
			}else{
				USART2->CR1 &= ~USART_CR1_TXEIE;
				USART2->CR1 |= USART_CR1_TCIE;
			}
		}

	}



}

/*
 * ADC map
 * 0 : C0, LOADCELL_1
 * 1 : C1, LOADCELL_2
 * 2 : C2, LOADCELL_3
 * 3 : C3, LOADCELL_4
 * 4 : C4, CURRENT_1
 * 5 : C5, CURRENT_2
 * 6 : B0, CURRENT_3
 * 7 : B1, CURRENT_4
 */
void ADC_IRQHandler()
{
	static uint8_t adc_ch = 0;

	adcEMA_pre[adc_ch] = adcEMA_cur[adc_ch];
	adcAvr_cur[adc_ch] = ADC1->DR;
	adcEMA_cur[adc_ch] = (int32_t)(VAL_EMAF_K * (float)adcAvr_cur[adc_ch]) + (int32_t)(VAL_EMAF_K_INV * (float)adcEMA_pre[adc_ch]);
	*adc[adc_ch] = adcEMA_cur[adc_ch];

	if(adc_ch < 7){
		adc_ch++;
		ADC1->CR2 |= ADC_CR2_SWSTART;
	}
	else{
		adc_ch = 0;
		adc_flag = OFF;
	}
}

  int main(int argc, char* argv[]){
	//SystemClock_Config();
	//set default value

	for(uint8_t i = 0; i < 4; i++){
		M[i].pos_goal 	=	(uint32_t*)(REGISTER + 		4 * i);
		M[i].velocity 	=	( int32_t*)(REGISTER + 16 + 4 * i);
		M[i].pos		=	(uint32_t*)(REGISTER + 32 + 4 * i);
		M[i].loadcell 	=	(uint16_t*)(REGISTER + 48 + 2 * i);
		M[i].current	=	(uint16_t*)(REGISTER + 56 + 2 * i);

		M[i].enc_h 		=	(uint16_t)((POS_INIT&0XFFFF0000)>>16);
		M[i].enc_l 		=	(uint16_t)POS_INIT&0X0000FFFF;

		*M[i].pos 		= 	((uint32_t)(M[i].enc_h) << 16) + (uint32_t)M[i].enc_l;
		*M[i].pos_goal 	=	*M[i].pos;
		M[i].pos_esum 	=	0;

		//for debugging (check pwm_out)(18.08.02)
		//M[i].p_output 	=	(int32_t*)(REGISTER + 90 + 4 * i);
	}

	M[0].get_enc = get_enc_M0;
	M[1].get_enc = get_enc_M1;
	M[2].get_enc = get_enc_M2;
	M[3].get_enc = get_enc_M3;

	M[0].set_motor = set_motor0;
	M[1].set_motor = set_motor1;
	M[2].set_motor = set_motor2;
	M[3].set_motor = set_motor3;

	for(uint8_t i = 0; i < 8; i++)
		adc[i] = (uint16_t*)(REGISTER + 48 + 2 * i);


	control_mode 		= 	(uint16_t*)(REGISTER + 64);
	LED_pwm 			= 	(uint16_t*)(REGISTER + 66);
	kp 					= 	(float*)(REGISTER + 68);
	ki 					= 	(float*)(REGISTER + 72);
	kd 					= 	(float*)(REGISTER + 76);
	hall_sensor 		= 	(uint16_t*)(REGISTER + 80);
	debug_cnt1			= 	(uint16_t*)(REGISTER + 82);
	debug_cnt2			= 	(uint16_t*)(REGISTER + 84);
	debug_cnt3			= 	(uint16_t*)(REGISTER + 86);
	debug_cnt4			= 	(uint16_t*)(REGISTER + 88);

	*kp 				=	DEFAULT_KP;
	*ki 				=	DEFAULT_KI;
	*kd 				=	DEFAULT_KD;


	//set register
	interrupt_init();
	GPIO_init();
	enc_init();
	PWM_init();
	timer_interrupt_init();
	USART_init();
	ADC_init();
	LED_init();


	for (;;)
    {
		// Add your code here.
    }
	return 0;
}

// v04_cpLED_ADCmf (v04에 v04_LED에 ADC MF 부분 카피)

int16_t get_enc_M0(void){
	return (int16_t)(0XFFFF - TIM2->CNT);
}

int16_t get_enc_M1(void){
	return (int16_t)TIM3->CNT;
}

int16_t get_enc_M2(void){
	return (int16_t)(0XFFFF - TIM4->CNT);
}

int16_t get_enc_M3(void){
	return (int16_t)TIM5->CNT;
}

/*
 *  check hall sensor and set pwm and direction
 */

void set_motor0(int32_t pwm){
	if(0 > pwm){
		M0_CW;
		M0_PWM = HALL_0_1 ? 0 : (uint32_t)-pwm;
	}
	else{
		M0_CCW;
		M0_PWM = HALL_0_0 ? 0 : (uint32_t)pwm;
	}
}
void set_motor1(int32_t pwm){
	if(0 > pwm){
		M1_CW;
		M1_PWM = HALL_1_1 ? 0 : (uint32_t)-pwm;
	}
	else{
		M1_CCW;
		M1_PWM = HALL_1_0 ? 0 : (uint32_t)pwm;
	}
}
void set_motor2(int32_t pwm){
	if(0 > pwm){
		M2_CW;
		M2_PWM = HALL_2_1 ? 0 : (uint32_t)-pwm;
	}
	else{
		M2_CCW;
		M2_PWM = HALL_2_0 ? 0 : (uint32_t)pwm;
	}
}
void set_motor3(int32_t pwm){
	if(0 > pwm){
		M3_CW;
		M3_PWM = HALL_3_1 ? 0 : (uint32_t)-pwm;
	}
	else{
		M3_CCW;
		M3_PWM = HALL_3_0 ? 0 : (uint32_t)pwm;
	}
}


/*
 * uart transmit
 */
void USART_transmit(uint8_t *data, uint16_t length){
	GPIOA->BSRR = GPIO_BSRR_BS_4;
	for(int i = 0; i < length; i ++)
	{
		while(!((USART2->SR & USART_SR_TXE) >> 7));

		USART2->DR = *(data + i);
	}
	while(!((USART2->SR & USART_SR_TC) >> 6));
	GPIOA->BSRR = GPIO_BSRR_BR_4;
}

inline void USART_TX_start(uint8_t *_data){
	for(uint8_t __i = 0; __i < 0xfe; __i++ );
	GPIOA->BSRR = GPIO_BSRR_BS_4;
	USART2->DR = *(_data);
	USART2->CR1 |= USART_CR1_TXEIE;
}

inline void USART_TX_data(uint8_t *_data){
	USART2->DR = *(_data);
}

inline void USART_TX_end(void){
	USART2->CR1 &= ~USART_CR1_TCIE;
	USART2->CR1 &= ~USART_CR1_TXEIE;
	GPIOA->BSRR = GPIO_BSRR_BR_4;
}


void interrupt_init(void){


	NVIC_SetPriorityGrouping(0x05);
	NVIC_SetPriority(USART2_IRQn, 3);
	NVIC_EnableIRQ(USART2_IRQn);

	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 4);
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	NVIC_SetPriority(TIM2_IRQn, 5);
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 5);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 5);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM5_IRQn, 5);
	NVIC_EnableIRQ(TIM5_IRQn);

	NVIC_SetPriority(ADC_IRQn , 6);
	NVIC_EnableIRQ(ADC_IRQn );

}

void GPIO_init(void){
	/*
	 * MOTOR direction pin
	 * 		M1 : B12, B13
	 * 		M2 : B14, B15
	 * 		M3 :  C6,  C7
	 * 		M4 :  C8,  C9
	 *
	 * TEST	pin : B8
	 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;


	GPIOB->MODER |= GPIO_MODER_MODER12_0|GPIO_MODER_MODER13_0|GPIO_MODER_MODER14_0|GPIO_MODER_MODER15_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_12|GPIO_OTYPER_OT_13|GPIO_OTYPER_OT_14|GPIO_OTYPER_OT_15);//Push-Pull or open drain
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12|GPIO_OSPEEDER_OSPEEDR13|GPIO_OSPEEDER_OSPEEDR14|GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12|GPIO_PUPDR_PUPDR13|GPIO_PUPDR_PUPDR14|GPIO_PUPDR_PUPDR15); //no pull up pull down

	GPIOC->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6|GPIO_OTYPER_OT_7|GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9);//Push-Pull or open drain
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7|GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9;
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6|GPIO_PUPDR_PUPDR7|GPIO_PUPDR_PUPDR8|GPIO_PUPDR_PUPDR9); //no pull up pull down

/*	GPIOB->MODER |= GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0|GPIO_MODER_MODER10_0;//0x00005555;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9|GPIO_OTYPER_OT_10);//Push-Pull or open drain
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9|GPIO_OSPEEDER_OSPEEDR10;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8|GPIO_PUPDR_PUPDR9|GPIO_PUPDR_PUPDR10); //no pull up pull down
//*/

	/*
	 * hall sensor pin
	 * 		h1 : C15
	 * 		h2 : C14
	 * 		h3 : C13
	 * 		h4 : C12
	 * 		h5 : C11
	 * 		h6 : C10
	 * 		h7 : A7
	 * 		h8 : A6
	 */
	GPIOA->MODER &= ~(GPIO_MODER_MODER6|GPIO_MODER_MODER7);
	GPIOC->MODER &= ~(GPIO_MODER_MODER10|GPIO_MODER_MODER11|GPIO_MODER_MODER12|GPIO_MODER_MODER13|GPIO_MODER_MODER14|GPIO_MODER_MODER15);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10|GPIO_OSPEEDER_OSPEEDR11|GPIO_OSPEEDER_OSPEEDR12|GPIO_OSPEEDER_OSPEEDR13|GPIO_OSPEEDER_OSPEEDR14|GPIO_OSPEEDER_OSPEEDR15;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR6_0|GPIO_PUPDR_PUPDR7_0; //pull up
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_0|GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR12_0|GPIO_PUPDR_PUPDR13_0|GPIO_PUPDR_PUPDR14_0|GPIO_PUPDR_PUPDR15_0; //pull up



}

// v04_cpLED_ADCmf (v04에 v04_LED에 ADC MF 부분 카피)

void USART_init(void){
	/*
	 * RS485 pin
	 * 		USART2
	 * 		TX : A2
	 * 		RX : A3
	 * 		TX enable : A4
	 */


	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->MODER |= GPIO_MODER_MODER4_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3|GPIO_OSPEEDER_OSPEEDR4;

	GPIOA->AFR[0] |= 0X00007700;


	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	USART2->CR2 = 0;
	USART2->CR3 = 0;
	USART2->BRR = (uint16_t)(84000000 /(2 * BAUDRATE));
}



void enc_init(void){
	/*
	 * encoder pin
	 * TIMER2
	 * M1_A : A15
	 * M1_B :  B3
	 * TIMER3
	 * M2_A :  B4
	 * M2_B :  B5
	 * TIMER4
	 * M3_A :  B6
	 * M3_B :  B7
	 * TIMER5
	 * M4_A :  A0
	 * M4_B :  A1
	 */

	//ENC_CH1
	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->AFR[1] |= 0X10000000;
	GPIOB->AFR[0] |= 0X00001000;

	//ENC_CH2
	GPIOB->MODER |= GPIO_MODER_MODER4_1;
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->AFR[0] |= 0X00020000;
	GPIOB->AFR[0] |= 0X00200000;

	//ENC_CH3
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->AFR[0] |= 0X02000000;
	GPIOB->AFR[0] |= 0X20000000;

	//ENC_CH4
	GPIOA->MODER |= GPIO_MODER_MODER0_1;
	GPIOA->MODER |= GPIO_MODER_MODER1_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
	GPIOA->AFR[0] |= 0X00000002;
	GPIOA->AFR[0] |= 0X00000020;



	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = TIM_CR1_CEN;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->SMCR = TIM_SMCR_SMS & (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	TIM2->CCMR1 = (TIM_CCMR1_CC1S & TIM_CCMR1_CC1S_0) | (TIM_CCMR1_CC2S & TIM_CCMR1_CC2S_0);
	TIM2->CCMR2 = 0;
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM2->ARR = 0XFFFF;
	TIM2->CNT = 0XFFFF - M[0].enc_l;

	//TIM2->CNT = 0XFFFF;

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 = TIM_CR1_CEN;
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->SMCR = TIM_SMCR_SMS & (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	TIM3->CCMR1 = (TIM_CCMR1_CC1S & TIM_CCMR1_CC1S_0) | (TIM_CCMR1_CC2S & TIM_CCMR1_CC2S_0);
	TIM3->CCMR2 = 0;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM3->ARR = 0XFFFF;
	TIM3->CNT = M[1].enc_l;

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CR1 = TIM_CR1_CEN;
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->SMCR = TIM_SMCR_SMS & (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	TIM4->CCMR1 = (TIM_CCMR1_CC1S & TIM_CCMR1_CC1S_0) | (TIM_CCMR1_CC2S & TIM_CCMR1_CC2S_0);
	TIM4->CCMR2 = 0;
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM4->ARR = 0XFFFF;
	TIM4->CNT = 0XFFFF - M[2].enc_l;

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->CR1 = TIM_CR1_CEN;
	TIM5->DIER |= TIM_DIER_UIE;
	TIM5->SMCR = TIM_SMCR_SMS & (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	TIM5->CCMR1 = (TIM_CCMR1_CC1S & TIM_CCMR1_CC1S_0) | (TIM_CCMR1_CC2S & TIM_CCMR1_CC2S_0);
	TIM5->CCMR2 = 0;
	TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM5->ARR = 0XFFFF;
	TIM5->CNT = M[3].enc_l;
}


void PWM_init(void){
	/*
	 * MOTOR PWM PIN
	 * M1 :  A8
	 * M2 :  A9
	 * M3 : A10
	 * M4 : A11
	 */

	GPIOA->MODER |= (GPIO_MODER_MODER8_1|GPIO_MODER_MODER9_1|GPIO_MODER_MODER10_1|GPIO_MODER_MODER11_1);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9|GPIO_OSPEEDER_OSPEEDR10|GPIO_OSPEEDER_OSPEEDR11);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9|GPIO_OTYPER_OT_10|GPIO_OTYPER_OT_11);//Push-Pull or open drain
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR8|GPIO_PUPDR_PUPDR9|GPIO_PUPDR_PUPDR10|GPIO_PUPDR_PUPDR11); //no pull up pull down


	GPIOA->AFR[1] |= 0x00001111;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->PSC = 0;//set prescaler

	TIM1->ARR = 999;//set auto reload register
	TIM1->EGR = TIM_EGR_UG;//update generation
	TIM1->CR1 |= TIM_CR1_ARPE| TIM_CR1_CEN;

	TIM1->CCER |= TIM_CCER_CC1E|TIM_CCER_CC1P|TIM_CCER_CC2E|TIM_CCER_CC2P|TIM_CCER_CC3E|TIM_CCER_CC3P|TIM_CCER_CC4E|TIM_CCER_CC4P;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M|TIM_CCMR1_OC1PE|TIM_CCMR1_OC2M|TIM_CCMR1_OC2PE;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M|TIM_CCMR2_OC3PE|TIM_CCMR2_OC4M|TIM_CCMR2_OC4PE;
	TIM1->BDTR = TIM_BDTR_MOE;
}

/************ TIM10 PWM set to control LED *************/
void LED_init(void){

	/***** Pin Setting *****
	LED pin: PB8
	TIM10_CH1: PB8
	AF3                   */

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB->MODER |= GPIO_MODER_MODER8_1;	//0x00020000U => PB8을 AF으로.
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR8;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_8;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0;		//0x00010000 is PB8 PU. (01:PU)

	GPIOB->AFR[1] |= 0x00000003;		//AF3 = 0b0011.

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	//count clock set. 	APB2: 84MHz. RCD_24 driver typ 200Hz.
	TIM10->PSC = 83;	//count clock = 1MHz.

	//period set.
	TIM10->ARR = 4999;	//200Hz.

	//initial duty-cycle set.
	TIM10->CCR1 = *LED_pwm;

	TIM10->CNT = 0;

	TIM10->EGR |= TIM_EGR_UG;
	TIM10->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;		//ARPE: Auto-reload Preload Enable, CEN: Count Enable.

	TIM10->CCER |= TIM_CCER_CC1E|TIM_CCER_CC1P;
	TIM10->CCMR1 |= TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE;	//OC1M: Output Compare 1 Mode, OC1PE: Output Compare 1 Preload Enable.(PWM mode 1)
}

// v04_cpLED_ADCmf (v04에 v04_LED에 ADC MF 부분 카피)

void ADC_init(void){

	// GPIOB 0,1 and GPIOC 4,5 ADC config

	GPIOB->MODER |= (GPIO_MODER_MODER0|GPIO_MODER_MODER1);//0x00000003;
	GPIOC->MODER |= (GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3|GPIO_MODER_MODER4|GPIO_MODER_MODER5);//0x00000003;
	//GPIOB->OTYPER = 0 ;//Push-Pull or open drain
	//GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0|GPIO_OSPEEDER_OSPEEDR1);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0|GPIO_PUPDR_PUPDR1); //no pull up pull down
	//GPIOC->OTYPER = 0 ;//Push-Pull or open drain
	//GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0|GPIO_OSPEEDER_OSPEEDR1|GPIO_OSPEEDER_OSPEEDR2|GPIO_OSPEEDER_OSPEEDR3|GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0|GPIO_PUPDR_PUPDR1|GPIO_PUPDR_PUPDR2|GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4|GPIO_PUPDR_PUPDR5); //no pull up pull down


	RCC->APB2ENR |= RCC_APB2LPENR_ADC1LPEN;
	ADC1->CR1 = ADC_CR1_DISCEN | ADC_CR1_SCAN | ADC_CR1_EOCIE;
	ADC1->CR1 &= ~ADC_CR1_DISCNUM;
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SQR1 |= ADC_SQR1_L_2|ADC_SQR1_L_1|ADC_SQR1_L_0;// L is the number of conversion. this mode is 'multi channel' and we used 4 channels so L should be 3

	ADC1->SMPR1 = 0;
	ADC1->SMPR2 = 0;

	/* this sampling time is 15cycle
	ADC1->SMPR1 |= ADC_SMPR1_SMP10_0;
	ADC1->SMPR1 |= ADC_SMPR1_SMP11_0;
	ADC1->SMPR1 |= ADC_SMPR1_SMP12_0;
	ADC1->SMPR1 |= ADC_SMPR1_SMP13_0;
	ADC1->SMPR1 |= ADC_SMPR1_SMP14_0;
	ADC1->SMPR1 |= ADC_SMPR1_SMP15_0;
	ADC1->SMPR2 |= ADC_SMPR2_SMP8_0;
	ADC1->SMPR2 |= ADC_SMPR2_SMP9_0;	//*/

	// this sampling time is 3cycle
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP10;
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP11;
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP12;
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP13;
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP14;
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP15;
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP8;
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP9;		//*/

	//sequence config, the order is 8,9,14,15 channel
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1|ADC_SQR3_SQ2|ADC_SQR3_SQ3|ADC_SQR3_SQ4|ADC_SQR3_SQ5|ADC_SQR3_SQ6);
	ADC1->SQR2 &= ~(ADC_SQR2_SQ7|ADC_SQR2_SQ8);
	ADC1->SQR3 |= ((ADC_SQR3_SQ1_1|ADC_SQR3_SQ1_3)|(ADC_SQR3_SQ2_0|ADC_SQR3_SQ2_1|ADC_SQR3_SQ2_3)|(ADC_SQR3_SQ3_2|ADC_SQR3_SQ3_3)|(ADC_SQR3_SQ4_0|ADC_SQR3_SQ4_2|ADC_SQR3_SQ4_2|ADC_SQR3_SQ4_3));// 8 9 14 15pin sequence
	ADC1->SQR3 |= ((ADC_SQR3_SQ5_1|ADC_SQR3_SQ5_2|ADC_SQR3_SQ5_3)|(ADC_SQR3_SQ6_0|ADC_SQR3_SQ6_1|ADC_SQR3_SQ6_2|ADC_SQR3_SQ6_3));
	ADC1->SQR2 |= ((ADC_SQR2_SQ7_3)|(ADC_SQR2_SQ8_0|ADC_SQR2_SQ8_3));

	ADC1->CR2 |= ADC_CR2_EOCS | ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_SWSTART;

}

void timer_interrupt_init(void){


	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	//TIM1->CR1 |= ;//set clock division, aligned mode, direction, counter enable,
	TIM9->ARR = 83;//set auto reload register
	TIM9->PSC = 99;//set prescaler
	TIM9->EGR = TIM_EGR_UG;//update generation
	TIM9->DIER |= TIM_DIER_UIE;//update interrupt enable
	TIM9->CR1 |= TIM_CR1_ARPE|TIM_CR1_URS|TIM_CR1_CEN;//counter enable

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

