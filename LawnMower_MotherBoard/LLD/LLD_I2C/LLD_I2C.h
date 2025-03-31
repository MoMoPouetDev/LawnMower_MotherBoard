/**
 * @file LLD_I2C.h
 * @author MVE
 * @brief Header file for I2C peripheral
 * @details
**/

#ifndef LLD_I2C_H_
#define LLD_I2C_H_
/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <util/twi.h>
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define F_CPU 8000000UL
#define SCL_CLOCK  400000UL
/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/
void LLD_I2C_Init(void);
uint8_t LLD_I2C_Read(uint8_t u8_slaveAddr, uint8_t u8_dataAddr);
void LLD_I2C_Write(uint8_t u8_slaveAddr, uint8_t u8_dataAddr, uint8_t u8_data);
/*--------------------------------------------------------------------------*/
/* ... END OF FILE...                                                      */
/*--------------------------------------------------------------------------*/
#endif /* LLD_I2C_H_ */
