/*
 * HAL_I2C.h
 *
 *  Created on: 12 sept. 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_I2C_HAL_I2C_H_
#define HAL_HAL_I2C_HAL_I2C_H_

#include <stdint.h>

void HAL_I2C_Init(void);
void HAL_I2C_CompassInit(void);
void HAL_I2C_AccelInit(void);
void HAL_I2C_Write(void);
uint8_t HAL_I2C_Read(uint8_t* , uint8_t* );
uint8_t HAL_I2C_ReadAccel(uint8_t* pu8_RxBuff, uint8_t* pu8_Size);
uint8_t HAL_I2C_ReadCompass(uint8_t* pu8_RxBuff, uint8_t* pu8_Size);

#endif /* HAL_HAL_I2C_HAL_I2C_H_ */
