/*
 * RUN_Init.c
 *
 *  Created on: 16 août 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <avr/interrupt.h>
#include "RUN_Task_Interface.h"
#include "RUN_Task.h"
#include "RUN_Init.h"
#include "RUN_Timer.h"
#include "RUN_ADC.h"
#include "RUN_GPIO.h"
#include "RUN_I2C.h"
#include "RUN_UART.h"
#include "RUN_PWM.h"
#include "RUN_FIFO.h"
#include "RUN_Sensors.h"
#include "RUN_Mower.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define RUN_INIT_WAIT_ONESEC 1000

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Init()
{
	RUN_Task_Interface_Init();
	RUN_GPIO_Init();
	RUN_I2C_Init();
	RUN_UART_Init();
	RUN_PWM_Init();
	RUN_FIFO_Init();
	RUN_Sensors_Init();
	RUN_Mower_Init();
	RUN_Timer_Init();
	RUN_ADC_Init();
	/*** Enable Interrupt */
	sei();
}

void RUN_Init_WaitSlave(void)
{
	while (RUN_Task_GetTickCounter() <= RUN_INIT_WAIT_ONESEC);
}
