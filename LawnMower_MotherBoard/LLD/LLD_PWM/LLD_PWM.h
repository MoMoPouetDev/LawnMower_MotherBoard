/**
 * @file LLD_PWM.h
 * @author ACR
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

#define SOURCE_CLK (150000000UL)

typedef enum
{
	LLD_PWM_PWM1_MODULE0,
	LLD_PWM_PWM1_MODULE1,
	LLD_PWM_PWM1_MODULE2,
	LLD_PWM_PWM1_MODULE3,
	LLD_PWM_PWM2_MODULE0,
	LLD_PWM_PWM2_MODULE1,
	LLD_PWM_PWM2_MODULE2,
	LLD_PWM_PWM2_MODULE3,
	LLD_PWM_PWM3_MODULE0,
	LLD_PWM_PWM3_MODULE1,
	LLD_PWM_PWM3_MODULE2,
	LLD_PWM_PWM3_MODULE3,
	LLD_PWM_PWM4_MODULE0,
	LLD_PWM_PWM4_MODULE1,
	LLD_PWM_PWM4_MODULE2,
	LLD_PWM_PWM4_MODULE3,
	LLD_PWM_NB
}typ_Lld_Pwm;

typedef enum
{
	LLD_PWM_CHANNEL_B,
	LLD_PWM_CHANNEL_A,
}typ_Lld_Pwm_Channel;

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/

void LLD_PWM_Init(typ_Lld_Pwm e_Pwm, typ_Lld_Pwm_Channel e_Channel, uint32_t u32_Frequency);
void LLD_PWM_Enable(typ_Lld_Pwm e_Pwm, typ_Lld_Pwm_Channel e_Channel, bool Enable);
void LLD_PWM_SetDutyCycle(typ_Lld_Pwm e_Pwm, typ_Lld_Pwm_Channel e_Channel, uint8_t u8_Dutycycle);

#endif /* LLD_PWM_H_ */






