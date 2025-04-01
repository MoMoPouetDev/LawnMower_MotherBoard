/**
 * @file LLD_PWM.h
 * @author MVE
 * @date 2019-03-26
 * @brief Header file for PWM peripheral
 * @details
**/

#ifndef LLD_PWM_H_
#define LLD_PWM_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/
void LLD_PWM_Init(void);
void LLD_PWM_Forward(uint8_t speed_ML, uint8_t speed_MR);
void LLD_PWM_Right(void);
void LLD_PWM_Left(void);
void LLD_PWM_Stop(void);

#endif /* LLD_PWM_H_ */