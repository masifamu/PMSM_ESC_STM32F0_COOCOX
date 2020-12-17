#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "sysclk.h"
#include "stm32f0xx_usart.h"
#include <stdio.h>
#include "stm32f0xx.h"
#include "PEPL_headers.h"

char TXBUFFER[50] = {'\0'};




int main(void)
{

	SetSysClockTo48();

	PMSM_ConfigureEngine();
	PMSM_SetEngineParameters();
	uint16_t count=0;

	while(1)
    {
		FIO_FLP(GPIOA,BLUE_LED);//Blue LED
		// Delay
		for (uint32_t i = 0; i < 0x0FFFFF; i++);

		sniprintf(TXBUFFER,20,"%d is %d\r\n",RCC_GetSYSCLKSource(),200);
		USARTSend(("hello\r\n"));
		USARTSend(TXBUFFER);

		//PMSM_UpdateEnginePWMWidth(100);


		if (1) {
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
		   	PMSM_SetPWM(100);
		}else {
			PMSM_SetPWM(0);
		}

    }
}

