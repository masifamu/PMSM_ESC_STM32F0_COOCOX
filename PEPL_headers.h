#ifndef _PEPL_HEADERS_LIB_H_
#define _PEPL_HEADERS_LIB_H_

#define HS_PINS							  GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7

//fast port macros
#define FIO_SET(port, pins)                port->BSRR = (pins)        //set pins
#define FIO_CLR(port, pins)                port->BRR = (pins)        //reset pins
#define FIO_FLP(port, pins)                port->ODR ^= (pins)        //flip pins
#define FIO_GET(port, pins)                ((port->IDR) & pins)        //read pins

//led pins
#define YELLOW_LED 							GPIO_Pin_3		//port B
#define GREEN_LED 							GPIO_Pin_4		//port B
#define BLUE_LED 							GPIO_Pin_12		//port A

//F/R pin
#define MOTOR_ROTATION_SELECT_PIN			GPIO_Pin_15		//on port A

//high side pwm channels
#define TIM1_CHANNEL_BH							TIM1->CCR1
#define TIM1_CHANNEL_GH							TIM1->CCR2
#define TIM1_CHANNEL_YH							TIM1->CCR3
#define TIM1_PWM_PERIOD							TIM1->ARR

#define PWM_PERIOD								2884

//select throttle/pot for speed control application.
#define THROTTLE
//#define POT

#ifdef POT
#define PMSM_ADC_START 							200
#define PMSM_ADC_STOP 							50
#define PMSM_ADC_MAX 							4000
#endif
#ifdef THROTTLE
#define PMSM_ADC_START 							1500
#define PMSM_ADC_STOP 							1450
#define PMSM_ADC_MAX 							4000
#endif

//Inverter Pins
#define MOS_YH 								TIM_Channel_3
#define MOS_GH 								TIM_Channel_2
#define MOS_BH 								TIM_Channel_1
#define MOS_YL 								GPIO_Pin_1
#define MOS_GL 								GPIO_Pin_0
#define MOS_BL 								GPIO_Pin_7

//hall sensor table bit positions
#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5



//speed measuring timer parameter
#define PMSM_SPEED_TIMER_PRESCALER				48 		//processor MHz
#define PMSM_SPEED_TIMER_PERIOD					0xFFFF 		// 65535

//motor rotation direction
#define PMSM_CW		0
#define PMSM_CCW	1

//function prototypes
void USARTSend(char *pucBuffer);
void USARTInit(void);
void PMSM_HallSensorsInit(void);
void PMSM_GPIOInit(void);
void PMSM_PWMTimerInit(void);
void PMSM_ConfigureEngine(void);
void PMSM_SetEngineParameters(void);
void PMSM_UpdateEnginePWMWidth(uint16_t pwmWidth);
void PMSM_SpeedTimerInit(void);
void PMSM_SinTimerInit(void);

uint8_t PMSM_HallSensorsGetPosition(void);
void PMSM_SetPWM(uint16_t PWM);
void PMSM_MotorStop(void);
uint8_t PMSM_MotorSpeedIsOK(void);
uint8_t	PMSM_GetState(uint8_t SensorsPosition);
void PMSM_MotorCommutation(uint16_t hallpos);
void PMSM_MotorLSCommutation(uint16_t hallLSpos);
void PMSM_MotorLS2Commutation(uint16_t sinTableIndex);
uint8_t PMSM_MotorIsRun(void);
void PMSM_MotorSetSpin(uint8_t spin);
void PMSM_MotorSetRun(void);
uint16_t PMSM_GetSpeed(void);
uint16_t PMSM_ADCToPWM(uint16_t ADC_VALUE);

#endif
