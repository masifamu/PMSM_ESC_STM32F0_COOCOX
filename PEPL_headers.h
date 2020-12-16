#ifndef _PEPL_HEADERS_LIB_H_
#define _PEPL_HEADERS_LIB_H_

#define HS_PINS							  GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7

//fast port macros
#define FIO_SET(port, pins)                port->BSRR = (pins)        //set pins
#define FIO_CLR(port, pins)                port->BRR = (pins)        //reset pins
#define FIO_FLP(port, pins)                port->ODR ^= (pins)        //flip pins
#define FIO_GET(port, pins)                ((port->IDR) & pins)        //read pins


void USARTSend(char *pucBuffer);
void USARTInit(void);
void PMSM_HallSensorsInit(void);
void PMSM_GPIOInit(void);
void PMSM_PWMTimerInit(void);
void PMSM_ConfigureEngine(void);
void PMSM_SetEngineParameters(void);
void PMSM_UpdateEnginePWMWidth(uint16_t pwmWidth);
#endif
