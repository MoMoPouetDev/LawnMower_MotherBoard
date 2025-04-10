/**
 * @file LLD_UART.c
 * @author ACR
 * @brief Specific UART driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <avr/io.h>

#include "LLD_UART.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD UART ...                                               */
/*--------------------------------------------------------------------------*/
#define F_CPU 8000000UL
#define BAUD 9600
#define BAUD_PRESCALE ((F_CPU/ (16UL*BAUD))-1)

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/
/**
* @brief		UART initialization
* @return		void
* @details
**/
void LLD_UART_Init(void)
{
    /***** UART BaudRate *****/
    UBRR0H = (unsigned char) (BAUD_PRESCALE>>8);
    UBRR0L = (unsigned char) BAUD_PRESCALE;
    
    /***** Autoriser Transmition et Reception *****/
    UCSR0B = (1<<TXEN0) | (1<<RXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); 
}

/**
* @brief		Receive data
* @param		u8_RxBuff : pointer on rx buffer
* @return		uint8_t
* @details
**/
uint8_t LLD_UART_Receive(uint8_t* pu8_rxBuff)
{
	static uint8_t _u8_uartState = 0;
    uint8_t u8_uartStatus = 0;

    switch (_u8_uartState)
    {
        case 0:
            if ((UCSR0A & (1<<RXC0)) != 0)
            {
                _u8_uartState++;
            }
            break;
        
        case 1:
            (*pu8_rxBuff) = UDR0;
            _u8_uartState = 0;
            u8_uartStatus = 1;
            break;
        
        default:
            _u8_uartState = 0;
            break;
    }

    return u8_uartStatus;
}

/**
* @brief		Send data
* @param		u8_TxBuff : pointer on tx buffer
* @return		uint8_t
* @details
**/
uint8_t LLD_UART_Send(uint8_t* pu8_txBuffer)
{
	static uint8_t _u8_uartState = 0;
    uint8_t u8_uartStatus = 0;

    switch (_u8_uartState)
    {
        case 0:
            if ((UCSR0A & (1<<UDRE0)) != 0)
            {
                _u8_uartState++;
            }
            break;
        
        case 1:
            UDR0 = (*pu8_txBuffer);
            _u8_uartState = 0;
            u8_uartStatus = 1;
            break;
        
        default:
            _u8_uartState = 0;
            break;
    }

    return u8_uartStatus;
}
