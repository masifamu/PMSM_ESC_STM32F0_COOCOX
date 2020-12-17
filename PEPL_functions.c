#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0Xx_tim.h"
#include "stm32f0xx.h"
#include "PEPL_headers.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"
#include <stdio.h>
#include <string.h>

/*..........................................................VARIABLE DEECLERATION STARTS HERE..................................................................*/
// BLDC motor steps tables
static const uint8_t PMSM_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

// BLDC motor backwared steps tables
static const uint8_t PMSM_BRIDGE_STATE_BACKWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,1	,	1,0	,	0,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

// Sin table
#define PMSM_SINTABLESIZE	192
static const uint8_t PMSM_SINTABLE [PMSM_SINTABLESIZE][3] =
{
		{0,       0,      221},
		{8,       0,      225},
		{17,      0,      229},
		{25,      0,      232},
		{33,      0,      236},
		{42,      0,      239},
		{50,      0,      241},
		{58,      0,      244},
		{66,      0,      246},
		{74,      0,      248},
		{82,      0,      250},
		{90,      0,      252},
		{98,      0,      253},
		{105,     0,      254},
		{113,     0,      254},
		{120,     0,      255},
		{128,     0,      255},
		{135,     0,      255},
		{142,     0,      254},
		{149,     0,      254},
		{155,     0,      253},
		{162,     0,      252},
		{168,     0,      250},
		{174,     0,      248},
		{180,     0,      246},
		{186,     0,      244},
		{192,     0,      241},
		{197,     0,      239},
		{202,     0,      236},
		{207,     0,      232},
		{212,     0,      229},
		{217,     0,      225},
		{221,     0,      221},
		{225,     0,      217},
		{229,     0,      212},
		{232,     0,      207},
		{236,     0,      202},
		{239,     0,      197},
		{241,     0,      192},
		{244,     0,      186},
		{246,     0,      180},
		{248,     0,      174},
		{250,     0,      168},
		{252,     0,      162},
		{253,     0,      155},
		{254,     0,      149},
		{254,     0,      142},
		{255,     0,      135},
		{255,     0,      127},
		{255,     0,      120},
		{254,     0,      113},
		{254,     0,      105},
		{253,     0,      98},
		{252,     0,      90},
		{250,     0,      82},
		{248,     0,      74},
		{246,     0,      66},
		{244,     0,      58},
		{241,     0,      50},
		{239,     0,      42},
		{236,     0,      33},
		{232,     0,      25},
		{229,     0,      17},
		{225,     0,      8},
		{221,     0,      0},
		{225,     8,      0},
		{229,     17,     0},
		{232,     25,     0},
		{236,     33,     0},
		{239,     42,     0},
		{241,     50,     0},
		{244,     58,     0},
		{246,     66,     0},
		{248,     74,     0},
		{250,     82,     0},
		{252,     90,     0},
		{253,     98,     0},
		{254,     105,    0},
		{254,     113,    0},
		{255,     120,    0},
		{255,     127,    0},
		{255,     135,    0},
		{254,     142,    0},
		{254,     149,    0},
		{253,     155,    0},
		{252,     162,    0},
		{250,     168,    0},
		{248,     174,    0},
		{246,     180,    0},
		{244,     186,    0},
		{241,     192,    0},
		{239,     197,    0},
		{236,     202,    0},
		{232,     207,    0},
		{229,     212,    0},
		{225,     217,    0},
		{221,     221,    0},
		{217,     225,    0},
		{212,     229,    0},
		{207,     232,    0},
		{202,     236,    0},
		{197,     239,    0},
		{192,     241,    0},
		{186,     244,    0},
		{180,     246,    0},
		{174,     248,    0},
		{168,     250,    0},
		{162,     252,    0},
		{155,     253,    0},
		{149,     254,    0},
		{142,     254,    0},
		{135,     255,    0},
		{128,     255,    0},
		{120,     255,    0},
		{113,     254,    0},
		{105,     254,    0},
		{98,      253,    0},
		{90,      252,    0},
		{82,      250,    0},
		{74,      248,    0},
		{66,      246,    0},
		{58,      244,    0},
		{50,      241,    0},
		{42,      239,    0},
		{33,      236,    0},
		{25,      232,    0},
		{17,      229,    0},
		{8,       225,    0},
		{0,       221,    0},
		{0,       225,    8},
		{0,       229,    17},
		{0,       232,    25},
		{0,       236,    33},
		{0,       239,    42},
		{0,       241,    50},
		{0,       244,    58},
		{0,       246,    66},
		{0,       248,    74},
		{0,       250,    82},
		{0,       252,    90},
		{0,       253,    98},
		{0,       254,    105},
		{0,       254,    113},
		{0,       255,    120},
		{0,       255,    128},
		{0,       255,    135},
		{0,       254,    142},
		{0,       254,    149},
		{0,       253,    155},
		{0,       252,    162},
		{0,       250,    168},
		{0,       248,    174},
		{0,       246,    180},
		{0,       244,    186},
		{0,       241,    192},
		{0,       239,    197},
		{0,       236,    202},
		{0,       232,    207},
		{0,       229,    212},
		{0,       225,    217},
		{0,       221,    221},
		{0,       217,    225},
		{0,       212,    229},
		{0,       207,    232},
		{0,       202,    236},
		{0,       197,    239},
		{0,       192,    241},
		{0,       186,    244},
		{0,       180,    246},
		{0,       174,    248},
		{0,       168,    250},
		{0,       162,    252},
		{0,       155,    253},
		{0,       149,    254},
		{0,       142,    254},
		{0,       135,    255},
		{0,       128,    255},
		{0,       120,    255},
		{0,       113,    254},
		{0,       105,    254},
		{0,       98,     253},
		{0,       90,     252},
		{0,       82,     250},
		{0,       74,     248},
		{0,       66,     246},
		{0,       58,     244},
		{0,       50,     241},
		{0,       42,     239},
		{0,       33,     236},
		{0,       25,     232},
		{0,       17,     229},
		{0,       8,      225}
};
char TXBUFFERF[50] = {'\0'};

// Phase correction table//check these values in the tables
static const uint8_t PMSM_STATE_TABLE_INDEX_FORWARD[8] = {0, 160, 32, 0, 96, 128, 64, 0};
static const uint8_t PMSM_STATE_TABLE_INDEX_BACKWARD[8] = {0, 32, 160, 0, 96, 64, 128, 0};
// Timing (points in sine table)
// sine table contains 192 items; 360/192 = 1.875 degrees per item
volatile static int8_t PMSM_Timing = 10; // 15 * 1.875 = 28.125 degrees


volatile uint8_t PMSM_MotorRunFlag = 0;
volatile uint8_t PMSM_MotorSpin = PMSM_CW;
uint8_t PMSM_STATE[6] = {0, 0, 0, 0, 0, 0};
volatile uint8_t	PMSM_Sensors = 0;
volatile uint8_t PMSM_SinTableIndex = 0;
volatile uint16_t PMSM_PWM = 0;
volatile uint16_t PMSM_Speed = 0;
volatile uint16_t PMSM_Speed_prev = 0;
volatile uint8_t PMSM_ModeEnabled = 0;
/*..........................................................VARIABLE DEECLERATION ENDS HERE..................................................................*/

/*..........................................................GPIO STARTS HERE..................................................................*/
void PMSM_GPIOInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	/* Configure Blue LED as output pushpull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PA15=F/R as input  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Configure Y=PB3 G=PB4 as output pushpull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PB0=GL,PB1=YL as output pull up */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	FIO_SET(GPIOB,GPIO_Pin_0|GPIO_Pin_1);

	/* Configure PA7=BL as output pull up */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	FIO_SET(GPIOA,GPIO_Pin_7);
}
/*..........................................................GPIO ENDS HERE..................................................................*/


/*..........................................................SPWM TIM1 STARTS HERE..................................................................*/
// Initialize Timer TIM1 & PWM output. Timer TIM1 generate 6-PWM outputs
void PMSM_PWMTimerInit(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// Initialize Tim1 PWM outputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2, 3 – set to PWM mode - all 3 outputs
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	//TIM_OCInitStructure.TIM_Pulse = 200; // initialize to zero output

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; ///
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; ///

	TIM_OCInitStructure.TIM_Pulse = 0; // initialize to zero output
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0; // initialize to zero output
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0; // initialize to zero output
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

	// DeadTime[ns] = value * (1/SystemCoreFreq) (on 72MHz: 7 is 98ns)
	//TIM_BDTRInitStructure.TIM_DeadTime = PMSM_NOL;

	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	// Break functionality
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_Cmd(TIM1, ENABLE);
	 // Enable motor timer main output (the bridge signals)
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/*..........................................................SPWM TIM1 ENDS HERE..................................................................*/
/*..........................................................PMSM CONTROL FUNCTIONS START HERE..................................................................*/
void PMSM_ConfigureEngine(void){
	PMSM_GPIOInit();
	PMSM_HallSensorsInit();
	USARTInit();
	PMSM_PWMTimerInit();
	PMSM_SpeedTimerInit();
	PMSM_SinTimerInit();
	PMSM_MotorStop();
}
void PMSM_SetEngineParameters(void){
	//TIM1 control
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);

	TIM1_PWM_PERIOD=2884;
	//TIM1_CHANNEL_YH=100;
	//TIM1_CHANNEL_GH=200;
	//TIM1_CHANNEL_BH=300;

	//variable control
	//TIM14 control
	//TIM16 control
}
void PMSM_UpdateEnginePWMWidth(uint16_t pwmWidth){
	PMSM_PWM=pwmWidth;
}

uint8_t PMSM_HallSensorsGetPosition(void) {
	return (uint8_t)((GPIOB->IDR) & (HS_PINS))>>5;
}

// Stop a motor
void PMSM_MotorStop(void)
{
	//set PWM widths to zero
	PMSM_SetPWM(0);

	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	//disable lower switches here

	//disable both timers
	TIM_Cmd(TIM14, DISABLE);
	TIM_Cmd(TIM16, DISABLE);
	PMSM_Speed = 0;
	PMSM_Speed_prev = 0;
	PMSM_MotorRunFlag = 0;
	PMSM_ModeEnabled = 0;
}

// Set PWM (same for all phases)
void PMSM_SetPWM(uint16_t PWM)
{
	if (PMSM_ModeEnabled == 0) {
		TIM1_CHANNEL_YH = PWM;
		TIM1_CHANNEL_GH = PWM;
		TIM1_CHANNEL_BH = PWM;
	}
	else {
		PMSM_PWM = PWM;
	}
}

uint8_t PMSM_MotorSpeedIsOK(void) {
	return ((PMSM_Speed_prev > 0) & (PMSM_Speed > 0));
}

// Get index in sine table based on the sensor data, the timing and the direction of rotor rotation
uint8_t	PMSM_GetState(uint8_t SensorsPosition) {
	int16_t index;

	if (PMSM_MotorSpin == PMSM_CW) {
		index = PMSM_STATE_TABLE_INDEX_FORWARD[SensorsPosition];
	}
	else {
		index = PMSM_STATE_TABLE_INDEX_BACKWARD[SensorsPosition];
	}

	index = index + (int16_t)PMSM_Timing;
	if (index > PMSM_SINTABLESIZE-1) {
		index = index - PMSM_SINTABLESIZE;
	}
	else {
		if (index < 0) {
			index = PMSM_SINTABLESIZE + index;
		}
	}

	return index;
}

void PMSM_MotorCommutation(uint16_t hallpos){

	if (PMSM_MotorSpin == PMSM_CW) {
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_FORWARD[hallpos], sizeof(PMSM_STATE));
	}
	else if(PMSM_MotorSpin == PMSM_CCW){
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_BACKWARD[hallpos], sizeof(PMSM_STATE));
	}
	// Disable if need
	if (!PMSM_STATE[UH]) TIM_CCxCmd(TIM1,MOS_YH,TIM_CCx_Disable);//TIM1->CCR3=0;
	if (!PMSM_STATE[UL]) FIO_SET(GPIOB,MOS_YL);//GPIOB->BSRR = 0x0002;//Y
	if (!PMSM_STATE[VH]) TIM_CCxCmd(TIM1,MOS_GH,TIM_CCx_Disable);//TIM1->CCR2=0;
	if (!PMSM_STATE[VL]) FIO_SET(GPIOB,MOS_GL);//GPIOB->BSRR = 0x0001;//G
	if (!PMSM_STATE[WH]) TIM_CCxCmd(TIM1,MOS_BH,TIM_CCx_Disable);//TIM1->CCR1=0;
	if (!PMSM_STATE[WL]) FIO_SET(GPIOA,MOS_BL);//GPIOA->BSRR = 0x0080;//B

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (PMSM_STATE[UH] & !PMSM_STATE[UL]) TIM_CCxCmd(TIM1,MOS_YH,TIM_CCx_Enable);//{ toUpdate = CH3; }
	if (PMSM_STATE[UL] & !PMSM_STATE[UH]) FIO_CLR(GPIOB,MOS_YL);//GPIOB->BRR = 0x0002;//Y
	if (PMSM_STATE[VH] & !PMSM_STATE[VL]) TIM_CCxCmd(TIM1,MOS_GH,TIM_CCx_Enable);//{	toUpdate = CH2; }
	if (PMSM_STATE[VL] & !PMSM_STATE[VH]) FIO_CLR(GPIOB,MOS_GL);//GPIOB->BRR = 0x0001;//G
	if (PMSM_STATE[WH] & !PMSM_STATE[WL]) TIM_CCxCmd(TIM1,MOS_BH,TIM_CCx_Enable);//{	toUpdate = CH1; }
	if (PMSM_STATE[WL] & !PMSM_STATE[WH]) FIO_CLR(GPIOA,MOS_BL);//GPIOA->BRR = 0x0080;//B
}
void PMSM_MotorLSCommutation(uint16_t hallLSpos){

	if (PMSM_MotorSpin == PMSM_CW) {
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_FORWARD[hallLSpos], sizeof(PMSM_STATE));
	}
	else if(PMSM_MotorSpin == PMSM_CCW){
		memcpy(PMSM_STATE, PMSM_BRIDGE_STATE_BACKWARD[hallLSpos], sizeof(PMSM_STATE));
	}
	// Disable if need
	if (!PMSM_STATE[UL]) FIO_SET(GPIOB,MOS_YL);//GPIOB->BSRR = 0x0002;//Y
	if (!PMSM_STATE[VL]) FIO_SET(GPIOB,MOS_GL);//GPIOB->BSRR = 0x0001;//G
	if (!PMSM_STATE[WL]) FIO_SET(GPIOA,MOS_BL);//GPIOA->BSRR = 0x0080;//B
	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (PMSM_STATE[UL] & !PMSM_STATE[UH]) FIO_CLR(GPIOB,MOS_YL);//GPIOB->BRR = 0x0002;//Y
	if (PMSM_STATE[VL] & !PMSM_STATE[VH]) FIO_CLR(GPIOB,MOS_GL);//GPIOB->BRR = 0x0001;//G
	if (PMSM_STATE[WL] & !PMSM_STATE[WH]) FIO_CLR(GPIOA,MOS_BL);//GPIOA->BRR = 0x0080;//B
}

void PMSM_MotorLS2Commutation(uint16_t sinTableIndex){

	if((sinTableIndex >= 133) & (sinTableIndex <= 187)){
		FIO_SET(GPIOB,MOS_GL);
		FIO_SET(GPIOA,MOS_BL);
		FIO_CLR(GPIOB,MOS_YL);
	}
	if((sinTableIndex >= 5) & (sinTableIndex <= 59)){
		FIO_SET(GPIOB,MOS_YL);
		FIO_SET(GPIOA,MOS_BL);
		FIO_CLR(GPIOB,MOS_GL);
	}
	if((sinTableIndex >= 69) & (sinTableIndex <= 123)){
		FIO_SET(GPIOB,MOS_GL);
		FIO_SET(GPIOB,MOS_YL);
		FIO_CLR(GPIOA,MOS_BL);
	}

}


void PMSM_MotorSetRun(void) {
	PMSM_MotorRunFlag = 1;
}

void PMSM_MotorSetSpin(uint8_t spin) {
	PMSM_MotorSpin = spin;
}

uint8_t PMSM_MotorIsRun(void) {
	return PMSM_MotorRunFlag;
}

uint16_t PMSM_GetSpeed(void) {
	return PMSM_Speed;
}

/*..........................................................PMSM_CONTROL FUNCTIONS END HERE..................................................................*/
/*..........................................................SPEED TIMER TIM14 STARTS HERE..................................................................*/
// Initialize TIM14. It is used to calculate the speed
void PMSM_SpeedTimerInit(void) {
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = PMSM_SPEED_TIMER_PRESCALER;
	TIMER_InitStructure.TIM_Period = PMSM_SPEED_TIMER_PERIOD;
	TIM_TimeBaseInit(TIM14, &TIMER_InitStructure);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM14, 0);
	//TIM_Cmd(TIM14, ENABLE);

	// NVIC Configuration
	// Enable the TIM14_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM14_IRQHandler(void) {
	if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
		// Overflow - the motor is stopped
		if (PMSM_MotorSpeedIsOK()) {
			PMSM_MotorStop();

			FIO_FLP(GPIOB,GREEN_LED);
		}
	}
}
/*..........................................................SPEED TIMER TIM14 ENDS HERE..................................................................*/

/*..........................................................SIN VALUE UPDATE TIM16 STARTS HERE..................................................................*/
// Initialize TIM16 which generate 3-phase sine signal
void PMSM_SinTimerInit(void) {
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = PMSM_SPEED_TIMER_PRESCALER;
	TIMER_InitStructure.TIM_Period = 0;
	TIM_TimeBaseInit(TIM16, &TIMER_InitStructure);
	TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);
	//TIM_SetCounter(TIM16, 0);
	//TIM_Cmd(TIM16, ENABLE);

	// NVIC Configuration
	// Enable the TIM16_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// Calculate 3-phase PWM and increment position in sine table
void TIM16_IRQHandler(void) {

	if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);

		// If time to enable PMSM mode
		if (PMSM_ModeEnabled == 0) {
			// Turn PWM outputs for working with sine wave
			TIM_CCxCmd(TIM1, MOS_YH, TIM_CCx_Enable);
			TIM_CCxCmd(TIM1, MOS_GH, TIM_CCx_Enable);
			TIM_CCxCmd(TIM1, MOS_BH, TIM_CCx_Enable);

			PMSM_ModeEnabled = 1;
		}

		// Calculate PWM for 3-phase
		if(PMSM_ModeEnabled == 1){
			TIM1_CHANNEL_YH = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][0]/255);
			TIM1_CHANNEL_GH = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][1]/255);
			TIM1_CHANNEL_BH = (uint16_t)((uint32_t)PMSM_PWM * PMSM_SINTABLE[PMSM_SinTableIndex][2]/255);
		}

		// Increment position in sine table
		PMSM_SinTableIndex++;
		if (PMSM_SinTableIndex > PMSM_SINTABLESIZE-1) {
			PMSM_SinTableIndex = 0;
		}

		PMSM_MotorLS2Commutation(PMSM_SinTableIndex);
	}
}
/*..........................................................SIN VALUE UPDATE TIM16 ENDS HERE..................................................................*/




/*..........................................................EXTI STARTS HERE..................................................................*/
// Configure GPIO, NVIC, EXTI for 3 Hall sensors
void PMSM_HallSensorsInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	// Enable clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// Init GPIO
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Tell system that you will use PB lines for EXTI
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);

	// Init NVIC
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

    // EXTI
    EXTI_InitStruct.EXTI_Line = EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStruct);
}
// Every time when hall sensors change state executed this IRQ handler
void EXTI4_15_IRQHandler(void) {
    if ((EXTI_GetITStatus(EXTI_Line5) | EXTI_GetITStatus(EXTI_Line6) | EXTI_GetITStatus(EXTI_Line7)) != RESET) {
    	// Clear interrupt flag
    	EXTI_ClearITPendingBit(EXTI_Line5);
    	EXTI_ClearITPendingBit(EXTI_Line6);
    	EXTI_ClearITPendingBit(EXTI_Line7);


    	PMSM_Sensors=PMSM_HallSensorsGetPosition();

    	PMSM_Speed_prev = PMSM_Speed;
    	//calculate the current speed of rotor by getting the counter value of TIM14
    	PMSM_Speed = TIM14->CNT;//get speed
    	TIM14->CR1|=TIM_CR1_CEN;//enable
    	TIM14->CNT = 0;//set

    	//setting to TIM16
    	if(PMSM_MotorSpeedIsOK()){
			TIM16->CNT = 0;
			TIM16->ARR = PMSM_Speed/32;
			TIM16->CR1 |= TIM_CR1_CEN;
    	}

    	// If Hall sensors value is valid
    	if ((PMSM_Sensors > 0 ) & (PMSM_Sensors < 7)) {
    		// Do a phase correction
    		PMSM_SinTableIndex = PMSM_GetState(PMSM_Sensors);
    	}

    	// If motor is started then used a block commutation
    	if (PMSM_ModeEnabled == 0) {
    		PMSM_MotorCommutation(PMSM_Sensors);
    	}//else{
    		//PMSM_MotorLSCommutation(PMSM_Sensors);
    	//}

    	FIO_FLP(GPIOB,YELLOW_LED);//Yellow LED


    	//USARTSend("from EXTI IRQ\r\n");
    	sniprintf(TXBUFFERF,20,"%d\r\n",PMSM_Speed);
    	USARTSend(TXBUFFERF);

    }
}
/*..........................................................EXTI ENDS HERE..................................................................*/



/*..........................................................UART START HERE..................................................................*/
void USARTInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configured as follow:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    /* Configure USART Tx, Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);

}
void USARTSend(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
/*..........................................................UART ENDS HERE..................................................................*/
