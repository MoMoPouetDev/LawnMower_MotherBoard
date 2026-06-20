/*
 * RUN_I2C.h
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_I2C_RUN_I2C_H_
#define RUN_RUN_I2C_RUN_I2C_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "HAL_I2C.h"
/*--------------------------------------------------------------------------*/
/*! ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
typedef enum
{
    E_I2C_USED_NONE,
    E_I2C_USED_ANGLES,
    E_I2C_USED_SLAVE,
}E_I2C_USED;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_I2C_Init(void);
void RUN_I2C_SetUsed(E_I2C_USED e_i2cUsed);
E_I2C_USED RUN_I2C_GetUsed(void);

#endif /* RUN_RUN_I2C_RUN_I2C_H_ */
