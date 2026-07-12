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
#ifndef DEBUG_UART
#define BAUD 9600
#else
#define BAUD 115200
#endif
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
    uint16_t u16_baudPrescale = 0;

    /* Arrondi correct : (F_CPU + 8*baud) / (16*baud) - 1
       au lieu d'une simple troncature qui décale l'UBRR de 1 */
    u16_baudPrescale = (uint16_t)(((F_CPU + (8UL * BAUD)) / (16UL * BAUD)) - 1UL);

    /***** UART BaudRate *****/
    UBRR0H = (unsigned char)(u16_baudPrescale >> 8);
    UBRR0L = (unsigned char)u16_baudPrescale;

    /***** Frame format: 8-bit, no parity, 1 stop bit *****/
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    /***** Autoriser Transmition et Reception *****/
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
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
