/*
 * RUN_I2C.c
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "RUN_I2C.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
E_I2C_USED ge_i2cUsed;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_I2C_Init(void)
{
	HAL_I2C_UnlockBus();
	HAL_I2C_Init();

	ge_i2cUsed = E_I2C_USED_NONE;
}

void RUN_I2C_InitSlave(void)
{
	HAL_I2C_AccelInit();
	HAL_I2C_CompassInit();
}

void RUN_I2C_SetUsed(E_I2C_USED e_i2cUsed)
{
	ge_i2cUsed = e_i2cUsed;
}

E_I2C_USED RUN_I2C_GetUsed(void)
{
	return ge_i2cUsed;
}

uint8_t RUN_I2C_CheckConsistency(void)
{
	uint8_t u8_errorFlag;

	u8_errorFlag = HAL_I2C_GetErrorFlag();
	if (u8_errorFlag != 0)
	{
		HAL_I2C_Reset();
	}

	return u8_errorFlag;
}
