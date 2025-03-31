/**
 * @file LLD_TIMER.h
 * @author ACR
 * @brief Header file for TIMER peripheral
 * @details
**/

#ifndef LLD_TIMER_H_
#define LLD_TIMER_H_

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

#define NB_LLD_TIMER_GPT	2
#define NB_LLD_TIMER_PIT	4

typedef enum
{
	LLD_TIMER_GPT1,
	LLD_TIMER_GPT2,
	LLD_TIMER_PIT0,
	LLD_TIMER_PIT1,
	LLD_TIMER_PIT2,
	LLD_TIMER_PIT3,
}typ_Lld_Timer;

typedef void (*lld_timer_transfer_callback_t)(void);

typedef enum
{
	LLD_TIMER_STATUS_ERROR,
	LLD_TIMER_STATUS_SUCCESS,
}typ_LldTimerStatus;

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/
typ_LldTimerStatus LLD_TIMER_GPT_Read(typ_Lld_Timer e_Number, uint32_t *pu32_TimerValue);
typ_LldTimerStatus LLD_TIMER_GPT_SetComparator(typ_Lld_Timer e_Number, uint32_t u32_cmpValue);

typ_LldTimerStatus LLD_TIMER_PIT_SetTimerInitialValue(typ_Lld_Timer e_Number, uint32_t u32_initValue);

void LLD_TIMER_Init(typ_Lld_Timer e_Number, uint8_t u8_EnableFreeRun);
void LLD_TIMER_InitGptInputCapture(typ_Lld_Timer e_Number, lld_timer_transfer_callback_t pf_callback);
void LLD_TIMER_EnableIrq(typ_Lld_Timer e_Number, uint8_t u8_Enable);
void LLD_TIMER_Start(typ_Lld_Timer e_Number);
void LLD_TIMER_Stop(typ_Lld_Timer e_Number);
void LLD_TIMER_SetCallback(typ_Lld_Timer e_Number, lld_timer_transfer_callback_t pf_callback);
void LLD_TIMER_GPT_ClearInputCaptureFlags(typ_Lld_Timer e_Number);

#endif /* LLD_TIMER_H_ */
