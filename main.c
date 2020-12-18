#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "sysclk.h"
#include "stm32f0xx_usart.h"
#include <stdio.h>
#include "stm32f0xx.h"
#include "PEPL_headers.h"
#include "adc_dma.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"

char TXBUFFER[50] = {'\0'};
extern uint16_t PMSM_Speed;
extern uint16_t PMSM_Speed_prev;



int main(void)
{

	SetSysClockTo48();
	ADC_DMA_init();
	PMSM_ConfigureEngine();
	PMSM_SetEngineParameters();
	uint16_t count=0;

	while(1)
    {
		// Delay
		//for (uint32_t i = 0; i < 0x0FFFFF; i++);

		sniprintf(TXBUFFER,40,"preSp=%d sp=%d ADC=%d w=%d\r\n",PMSM_Speed_prev,PMSM_Speed,(ADCBuffer[0] & 0xFFF0),PMSM_ADCToPWM((ADCBuffer[0] & 0xFFF0)));
		//USARTSend(("hello\r\n"));
		USARTSend(TXBUFFER);

		//PMSM_UpdateEnginePWMWidth(100);


		if ((ADCBuffer[0] & 0xFFF0) > PMSM_ADC_START) {
		    // If Motor Is not run
		    if (PMSM_MotorIsRun() == 0) {
		    	// Start motor
		    	// Check Reverse pin
		    	if (FIO_GET(GPIOA,MOTOR_ROTATION_SELECT_PIN) != 0) {
		    		// Forward
		    		PMSM_MotorSetSpin(PMSM_CW);
		    	}else {
		    		// Backward
		    		PMSM_MotorSetSpin(PMSM_CCW);
		    	}
		    	PMSM_MotorCommutation(PMSM_HallSensorsGetPosition());
		    	PMSM_MotorSetRun();
		    }
		   	PMSM_SetPWM(PMSM_ADCToPWM((ADCBuffer[0] & 0xFFF0)));
		   	FIO_SET(GPIOA,BLUE_LED);//Blue LED
		}else {
			PMSM_SetPWM(0);
			FIO_CLR(GPIOA,BLUE_LED);//Blue LED
		}

    }
}

