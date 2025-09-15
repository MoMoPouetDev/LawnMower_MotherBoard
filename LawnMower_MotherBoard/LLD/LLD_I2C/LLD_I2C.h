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
#define F_CPU 20000000UL
#define SCL_CLOCK  400000UL
#define LLD_I2C_BUFFER_SIZE 32

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/
void LLD_I2C_UnlockBus(void);
void LLD_I2C_Init(void);
void LLD_I2C_InitCompass(uint8_t addrSlave);
void LLD_I2C_InitAccel(uint8_t addrSlave);
uint8_t LLD_I2C_Read(uint8_t addrSlave, uint8_t addrData);
void LLD_I2C_Write(uint8_t addrSlave, uint8_t addrData, uint8_t data);
void LLD_I2C_Reset(void);
uint8_t LLD_I2C_GetErrorFlag(void);
/*--------------------------------------------------------------------------*/
/* ... END OF FILE...                                                      */
/*--------------------------------------------------------------------------*/
#endif /* LLD_I2C_H_ */
