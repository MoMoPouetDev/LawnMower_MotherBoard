/**
 * @file LLD_GPIO.c
 * @author ACR
 * @brief Specific GPIO driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>

#include "LLD_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES GPIO ...                                                   */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD GPIO ...                                               */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		GPIO initialization
* @details
**/
void LLD_GPIO_Init(void)
{
	/***** PORT B *****/
	DDRB = 0x00;
	//DDRB |= (0<<DDB0); // Bouton Poussoir Stop
	//DDRB |= (1<<DDB1) | (1<<DDB2) | (1<<DDB4); // Bumper Left - Center - Right
	DDRB |= (1<<DDB3); // PWM Moteur 2
	DDRB |= (1<<DDB5); // Commande Moteur Lame Enable 
	//DDRB |= (1<<DDB6) | (1<<DDB7); // XTAL

	PORTB = 0x00;
	PORTB |= (1<<PORTB0); // Pull-Up Bouton Poussoir
	PORTB |= (1<<PORTB1) | (1<<PORTB2) | (1<<PORTB4); // Bumper Left - Center - Right Pull Up
	//PORTB |= (1<<PORTB3); // Moteur 2 Avant
	//PORTB |= (1<<PORTB5); // Moteur Lame Enable
	//PORTB |= (1<<PORTB6); // XTAL
	//PORTB |= (1<<PORTB7); // XTAL

	/***** PORT C *****/
	DDRC = 0x00;
	//DDRC |= (1<<DDC0) | (1<<DDC1); // ADC - Detection cable droite et gauche
	//DDRC |= (1<<DDC2) | (1<<DDC3); // No use - current sensor both
	//DDRC |= (1<<DDC4) | (1<<DDC5); // Config I2C SDA - SCL
	//DDRC |= (1<<DDC6); // reset

	PORTC = 0x00;
	//PORTC &= ~(1<<PORTC0) & ~(1<<PORTC1); // ADC - No Pull-Up
	PORTC |= (1<<PORTC2) | (1<<PORTC3); // TBD Pull Up
	//PORTC &= ~(1<<PORTC4) & ~(1<<PORTC5); // I2C - Force à 0
	PORTC |= (1<<PORTC6); // reset

	/***** PORT D *****/
	DDRD = 0x00;
	DDRD |= (1<<DDD1); //| (0<<DDD0); // UART - TXD - RXD
	DDRD |= (1<<DDD2) | (1<<DDD3); // Commande Avant Moteur 2 - Commande Arriere Moteur 2
	DDRD |= (1<<DDD4) | (1<<DDD5) | (1<<DDD6); // Commande Avant Moteur 1 - Commande Arriere Moteur 1 - PWM Moteur 1
	//DDRD |= (0<<DDD7); // Bouton Poussoir Start

	PORTD = 0x00;
	PORTD |= (1<<PORTD0); //| (1<<PORTD1); // UART - RX Pull-Up - TX
	//PORTD |= (1<<PORTD2) | (1<<PORTD3); // Commande Avant Moteur 2 - Commande Arriere Moteur 2 - Force à 0
	//PORTD |= (1<<PORTD4) | (1<<PORTD5) | (1<<PORTD6); // Commande Avant Moteur 1 - Commande Arriere Moteur 1 - PWM Moteur 1
	PORTD |= (1<<PORTD7); // Pull-Up Bouton Poussoir
}

/**
* @brief		Sets the output level of GPIO pin to logic 1
* @param[in]	e_Gpio Name of GPIO pin to set the output level
* @return		void
* @details
**/
void LLD_GPIO_WritePin(GPIO e_Gpio)
{
	switch (e_Gpio)
	{
		case E_MOTOR_BLADE_ENABLE:
			PORTB |= (1<<PORTB5);
			break;

		case E_MOTOR_ONE_FORWARD_ENABLE:
			PORTD |= (1<<PORTD4);
			break;

		case E_MOTOR_ONE_BACKWARD_ENABLE:
			PORTD |= (1<<PORTD5);
			break;

		case E_MOTOR_TWO_FORWARD_ENABLE:
			PORTD |= (1<<PORTD2);
			break;

		case E_MOTOR_TWO_BACKWARD_ENABLE:
			PORTD |= (1<<PORTD3);
			break;
		
		default:
			break;
	}
}

/**
* @brief		Sets the output level of GPIO pin to logic 0
* @param[in]	e_Gpio Name of GPIO pin to set the output level
* @return		void
* @details
**/
void LLD_GPIO_ClearPin(GPIO e_Gpio)
{
	switch (e_Gpio)
	{
		case E_MOTOR_BLADE_ENABLE:
			PORTB &= ~(1<<PORTB5);
			break;

		case E_MOTOR_ONE_FORWARD_ENABLE:
			PORTD &= ~(1<<PORTD4);
			break;

		case E_MOTOR_ONE_BACKWARD_ENABLE:
			PORTD &= ~(1<<PORTD5);
			break;

		case E_MOTOR_TWO_FORWARD_ENABLE:
			PORTD &= ~(1<<PORTD2);
			break;

		case E_MOTOR_TWO_BACKWARD_ENABLE:
			PORTD &= ~(1<<PORTD3);
			break;
		
		default:
			break;
	}
}

/**
* @brief		Reads the current input value of the GPIO pin.
* @param[in]	e_Gpio Name of GPIO pin to read input value
* @return		Input value of GPIO pin
* @details
**/
uint8_t LLD_GPIO_ReadPin(GPIO e_Gpio)
{
	uint8_t u8_value = 0;
	switch (e_Gpio)
	{
		case E_STOP_BUTTON:
			u8_value = PINB & (1<<PINB0);
			break;
		case E_START_BUTTON:
			u8_value = PIND & (1<<PIND7);
			break;

		case E_LEFT_BUMPER:
			u8_value = PINB & (1<<PINB1);
			break;

		case E_CENTER_BUMPER:
			u8_value = PINB & (1<<PINB2);
			break;

		case E_RIGHT_BUMPER:
			u8_value = PINB & (1<<PINB4);
			break;
		
		default:
			break;
	}
	return u8_value;
}

/**
* @brief		Reverses the current output logic of the GPIO pin
* @param[in]	e_Gpio Name of GPIO pin to reverse value
* @return		void
* @details
**/
void LLD_GPIO_Toggle(GPIO e_Gpio)
{
	
}
