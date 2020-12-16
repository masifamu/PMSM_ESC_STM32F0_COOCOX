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
		FIO_FLP(GPIOA,GPIO_Pin_12);//Blue LED
		// Delay
		for (uint32_t i = 0; i < 0x0FFFFF; i++);

		sniprintf(TXBUFFER,20,"%d is %d\r\n",RCC_GetSYSCLKSource(),200);
		USARTSend(("hello\r\n"));
		USARTSend(TXBUFFER);


		PMSM_UpdateEnginePWMWidth(++count);
		if(count>=499) count=0;

    }
}

