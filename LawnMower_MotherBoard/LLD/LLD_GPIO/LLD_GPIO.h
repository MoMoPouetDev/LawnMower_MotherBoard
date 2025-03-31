/**
 * @file LLD_GPIO.h
 * @author ACR
 * @brief Header file for GPIO peripheral
 * @details
**/

#ifndef LLD_GPIO_H_
#define LLD_GPIO_H_

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <avr/io.h>
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
typedef enum
{
	E_STOP_BUTTON,
	E_START_BUTTON,

	E_LEFT_BUMPER,
	E_CENTER_BUMPER,
	E_RIGHT_BUMPER,

	E_MOTOR_BLADE_ENABLE,
	E_MOTOR_ONE_FORWARD_ENABLE,
	E_MOTOR_ONE_BACKWARD_ENABLE,
	E_MOTOR_TWO_FORWARD_ENABLE,
	E_MOTOR_TWO_BACKWARD_ENABLE
}GPIO;

typedef void (*lld_gpio_transfer_callback_t)(void);

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/

void LLD_GPIO_Init(void);
void LLD_GPIO_WritePin(GPIO e_Gpio);
void LLD_GPIO_ClearPin(GPIO e_Gpio);
uint8_t LLD_GPIO_ReadPin(GPIO e_Gpio);
void LLD_GPIO_Toggle(GPIO e_Gpio);

#endif /* LLD_GPIO_H_ */
