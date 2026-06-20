/**
 * @file LLD_I2C.c
 * @author MVE
 * @brief Specific I2C driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <string.h> 
#include <avr/io.h>
#include <avr/interrupt.h>

#include "LLD_I2C.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD I2C ...                                                */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _LLD_I2C_Write(uint8_t addrSlave, uint8_t twi_read_write);
static void _LLD_I2C_WriteData(uint8_t dataToSend);
static void _LLD_I2C_Start(void);
static void _LLD_I2C_RepeatStart(void);
static uint8_t _LLD_I2C_ReadAck(void);
static uint8_t _LLD_I2C_ReadNack(void);
static void _LLD_I2C_Stop(void);
/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*! @brief D�bloque le bus I2C si SDA est maintenu bas
 *         en g�n�rant 9 impulsions manuelles sur SCL + STOP
 */
/*--------------------------------------------------------------------------*/
/**
* @brief		I2C initialization
* @return		void
* @details
**/
void LLD_I2C_Init(void)
{
    TWSR = 0;
    TWBR  = (( F_CPU  / SCL_CLOCK ) - 16 ) / 2; //- 400kHz
	TWCR = (1 << TWEN);
}

/**
* @brief		I2C start condition for Compass HMC5883
* @return		void
* @details
**/
void LLD_I2C_InitCompass(uint8_t addrSlave)
{
	_LLD_I2C_Start();
	_LLD_I2C_Write(addrSlave, TW_WRITE);
	_LLD_I2C_WriteData(0x00);
	_LLD_I2C_WriteData(0x70);
	_LLD_I2C_RepeatStart();
	_LLD_I2C_Write(addrSlave, TW_WRITE);
	_LLD_I2C_WriteData(0x01);
	_LLD_I2C_WriteData(0xE0);
	_LLD_I2C_RepeatStart();
	_LLD_I2C_Write(addrSlave, TW_WRITE);
	_LLD_I2C_WriteData(0x02);
	_LLD_I2C_WriteData(0x00);
	_LLD_I2C_Stop();
}

void LLD_I2C_InitAccel(uint8_t addrSlave)
{
	_LLD_I2C_Start();
	_LLD_I2C_Write(addrSlave, TW_WRITE);
	_LLD_I2C_WriteData(0x2D);
	_LLD_I2C_WriteData(0x08);
	_LLD_I2C_Stop();
}

/**
* @brief		Write data to slave
* @param		e_I2c : I2C number
* @param		u8_SlaveAddress : slave address
* @param		u8_DataAddress : data address
* @return		void
* @details
**/
void LLD_I2C_Write(uint8_t addrSlave, uint8_t addrData, uint8_t data) 
{

    _LLD_I2C_Start();
    _LLD_I2C_Write(addrSlave, TW_WRITE);
    _LLD_I2C_WriteData(addrData);
    _LLD_I2C_WriteData(data);
    _LLD_I2C_Stop();
}

/**
* @brief		Read data from slave
* @param		e_I2c : I2C number
* @param		u8_SlaveAddress : slave address (7 bits)    
* @param		u8_DataAddress : data address
* @return		uint8_t
* @details
**/
uint8_t LLD_I2C_Read(uint8_t addrSlave, uint8_t addrData) 
{
    uint8_t receivedData = 0;
	    
    _LLD_I2C_Start();
    _LLD_I2C_Write(addrSlave, TW_WRITE);
    _LLD_I2C_WriteData(addrData);
    
    _LLD_I2C_RepeatStart();
    _LLD_I2C_Write(addrSlave, TW_READ);
    receivedData = _LLD_I2C_ReadNack();
    
    _LLD_I2C_Stop();
        
    return receivedData;
}

/**
 * @brief 
 * 
 * @param addrSlave 
 * @param twi_read_write 
 */
static void _LLD_I2C_Write(uint8_t addrSlave, uint8_t twi_read_write)
{	
	TWDR = addrSlave + twi_read_write;
	TWCR = (1<<TWEN) | (1<<TWINT);

	while (!(TWCR & (1 << TWINT)));
		
	if (twi_read_write) 
	{
		while ((TWSR & 0xF8) != TW_MR_SLA_ACK);
	} else 
	{
		while ((TWSR & 0xF8) != TW_MT_SLA_ACK);
	}
}

/**
 * @brief 
 * 
 * @param dataToSend 
 */
static void _LLD_I2C_WriteData(uint8_t dataToSend)
{
	TWDR = dataToSend;
	TWCR = (1<<TWEN) | (1<<TWINT);

	while (!(TWCR & (1 << TWINT)));
	while ((TWSR & 0xF8) != TW_MT_DATA_ACK);
}

static void _LLD_I2C_Start(void)
{	
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);

    while (!(TWCR & (1 << TWINT)));
    while ((TWSR & 0xF8) != TW_START);
}

static void _LLD_I2C_RepeatStart(void)
{
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);

    while (!(TWCR & (1 << TWINT)));	
    while ((TWSR & 0xF8) != TW_REP_START);
}

static uint8_t _LLD_I2C_ReadAck(void)
{
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	while ((TWSR & 0xF8) != TW_MR_DATA_ACK);
	return TWDR;
}

static uint8_t _LLD_I2C_ReadNack(void)
{
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	while ((TWSR & 0xF8) != TW_MR_DATA_NACK);
	return TWDR;
}

static void _LLD_I2C_Stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

	while ((TWCR & (1 << TWSTO)));
}


