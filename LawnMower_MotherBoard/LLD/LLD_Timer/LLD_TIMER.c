/**
 * @file LLD_TIMER.c
 * @author ACR
 * @brief Specific TIMER driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_TIMER_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "MIMXRT1062.h"
#include "MIMXRT1062_features.h"
#include "fsl_clock.h"
#include "LLD_TIMER.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES GPT ...                                                    */
/*--------------------------------------------------------------------------*/

/*!
 * @brief List of clock sources
 * @note Actual number of clock sources is SoC dependent
 */
typedef enum _gpt_clock_source
{
    kGPT_ClockSource_Off      = 0U, /*!< GPT Clock Source Off.*/
    kGPT_ClockSource_Periph   = 1U, /*!< GPT Clock Source from Peripheral Clock.*/
    kGPT_ClockSource_HighFreq = 2U, /*!< GPT Clock Source from High Frequency Reference Clock.*/
    kGPT_ClockSource_Ext      = 3U, /*!< GPT Clock Source from external pin.*/
    kGPT_ClockSource_LowFreq  = 4U, /*!< GPT Clock Source from Low Frequency Reference Clock.*/
    kGPT_ClockSource_Osc      = 5U, /*!< GPT Clock Source from Crystal oscillator.*/
} gpt_clock_source_t;

/*! @brief List of input capture channel number. */
typedef enum _gpt_input_capture_channel
{
    kGPT_InputCapture_Channel1 = 0U, /*!< GPT Input Capture Channel1.*/
    kGPT_InputCapture_Channel2 = 1U, /*!< GPT Input Capture Channel2.*/
} gpt_input_capture_channel_t;

/*! @brief List of input capture operation mode. */
typedef enum _gpt_input_operation_mode
{
    kGPT_InputOperation_Disabled = 0U, /*!< Don't capture.*/
    kGPT_InputOperation_RiseEdge = 1U, /*!< Capture on rising edge of input pin.*/
    kGPT_InputOperation_FallEdge = 2U, /*!< Capture on falling edge of input pin.*/
    kGPT_InputOperation_BothEdge = 3U, /*!< Capture on both edges of input pin.*/
} gpt_input_operation_mode_t;

/*! @brief List of output compare channel number. */
typedef enum _gpt_output_compare_channel
{
    kGPT_OutputCompare_Channel1 = 0U, /*!< Output Compare Channel1.*/
    kGPT_OutputCompare_Channel2 = 1U, /*!< Output Compare Channel2.*/
    kGPT_OutputCompare_Channel3 = 2U, /*!< Output Compare Channel3.*/
} gpt_output_compare_channel_t;

/*! @brief List of output compare operation mode. */
typedef enum _gpt_output_operation_mode
{
    kGPT_OutputOperation_Disconnected = 0U, /*!< Don't change output pin.*/
    kGPT_OutputOperation_Toggle       = 1U, /*!< Toggle output pin.*/
    kGPT_OutputOperation_Clear        = 2U, /*!< Set output pin low.*/
    kGPT_OutputOperation_Set          = 3U, /*!< Set output pin high.*/
    kGPT_OutputOperation_Activelow    = 4U, /*!< Generate a active low pulse on output pin.*/
} gpt_output_operation_mode_t;

/*! @brief List of GPT interrupts */
typedef enum _gpt_interrupt_enable
{
    kGPT_OutputCompare1InterruptEnable = GPT_IR_OF1IE_MASK, /*!< Output Compare Channel1 interrupt enable*/
    kGPT_OutputCompare2InterruptEnable = GPT_IR_OF2IE_MASK, /*!< Output Compare Channel2 interrupt enable*/
    kGPT_OutputCompare3InterruptEnable = GPT_IR_OF3IE_MASK, /*!< Output Compare Channel3 interrupt enable*/
    kGPT_InputCapture1InterruptEnable  = GPT_IR_IF1IE_MASK, /*!< Input Capture Channel1 interrupt enable*/
    kGPT_InputCapture2InterruptEnable  = GPT_IR_IF2IE_MASK, /*!< Input Capture Channel1 interrupt enable*/
    kGPT_RollOverFlagInterruptEnable   = GPT_IR_ROVIE_MASK, /*!< Counter rolled over interrupt enable*/
} gpt_interrupt_enable_t;

/*! @brief Status flag. */
typedef enum _gpt_status_flag
{
    kGPT_OutputCompare1Flag = GPT_SR_OF1_MASK, /*!< Output compare channel 1 event.*/
    kGPT_OutputCompare2Flag = GPT_SR_OF2_MASK, /*!< Output compare channel 2 event.*/
    kGPT_OutputCompare3Flag = GPT_SR_OF3_MASK, /*!< Output compare channel 3 event.*/
    kGPT_InputCapture1Flag  = GPT_SR_IF1_MASK, /*!< Input Capture channel 1 event.*/
    kGPT_InputCapture2Flag  = GPT_SR_IF2_MASK, /*!< Input Capture channel 2 event.*/
    kGPT_RollOverFlag       = GPT_SR_ROV_MASK, /*!< Counter reaches maximum value and rolled over to 0 event.*/
} gpt_status_flag_t;

/*! @brief Structure to configure the running mode. */
typedef struct _gpt_init_config
{
    bool enableFreeRun;             /*!< true: FreeRun mode, false: Restart mode. */
} gpt_config_t;


/*--------------------------------------------------------------------------*/
/* ... DATATYPES PIT ...                                                    */
/*--------------------------------------------------------------------------*/

#define PIT_SOURCE_CLOCK 60000000UL

/*!
 * @brief List of PIT channels
 * @note Actual number of available channels is SoC dependent
 */
typedef enum _pit_chnl
{
    kPIT_Chnl_0 = 0U, /*!< PIT channel number 0*/
    kPIT_Chnl_1,      /*!< PIT channel number 1 */
    kPIT_Chnl_2,      /*!< PIT channel number 2 */
    kPIT_Chnl_3,      /*!< PIT channel number 3 */
} pit_chnl_t;

/*! @brief List of PIT interrupts */
typedef enum _pit_interrupt_enable
{
    kPIT_TimerInterruptEnable = PIT_TCTRL_TIE_MASK, /*!< Timer interrupt enable*/
} pit_interrupt_enable_t;

/*! @brief List of PIT status flags */
typedef enum _pit_status_flags
{
    kPIT_TimerFlag = PIT_TFLG_TIF_MASK, /*!< Timer flag */
} pit_status_flags_t;

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD TIMER ...                                              */
/*--------------------------------------------------------------------------*/

/*! @brief LLD CAN handle structure definition. */
typedef struct
{
    lld_timer_transfer_callback_t callback; /*!< Callback function. */
}lld_timer_handle_t;

typedef struct
{
	GPT_Type * ps_Base;
	IRQn_Type e_Irq;
	lld_timer_handle_t s_ldd_timer_handle;

}typ_Lld_Timer_Gpt_manager;

typ_Lld_Timer_Gpt_manager ts_lld_timer_gpt_manager[NB_LLD_TIMER_GPT];

typedef struct
{
	PIT_Type * ps_Base;
	pit_chnl_t e_Channel;
	IRQn_Type e_Irq;
	lld_timer_handle_t s_ldd_timer_handle;

}typ_Lld_Timer_Pit_manager;

typ_Lld_Timer_Pit_manager ts_lld_timer_pit_manager[NB_LLD_TIMER_PIT];


/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

void LLD_TIMER_GPT_Init(GPT_Type *base, const gpt_config_t *initConfig);
static inline void LLD_TIMER_GPT_SoftwareReset(GPT_Type *base);
static inline void LLD_TIMER_GPT_StartTimer(GPT_Type *base);
static inline void LLD_TIMER_GPT_StopTimer(GPT_Type *base);
static inline uint32_t LLD_TIMER_GPT_GetCurrentTimerCount(GPT_Type *base);
static inline void LLD_TIMER_GPT_SetInputOperationMode(GPT_Type *base,
												gpt_input_capture_channel_t channel,
												gpt_input_operation_mode_t mode);
static inline void LLD_TIMER_GPT_SetOutputOperationMode(GPT_Type *base,
                                              gpt_output_compare_channel_t channel,
                                              gpt_output_operation_mode_t mode);
static inline void LLD_TIMER_GPT_SetOutputCompareValue(GPT_Type *base, gpt_output_compare_channel_t channel, uint32_t value);
static inline void LLD_TIMER_GPT_EnableInterrupts(GPT_Type *base, uint32_t mask);
static inline void LLD_TIMER_GPT_DisableInterrupts(GPT_Type *base, uint32_t mask);
static inline uint32_t LLD_TIMER_GPT_GetStatusFlags(GPT_Type *base, gpt_status_flag_t flags);
static inline void LLD_TIMER_GPT_ClearStatusFlags(GPT_Type *base, gpt_status_flag_t flags);
static uint8_t LLD_TIMER_GPT_GetOffset(typ_Lld_Timer e_Number);
void GPT1_DriverIRQHandler(void);
void GPT2_DriverIRQHandler(void);
void LLD_TIMER_GPT_HandleIRQ(typ_Lld_Timer e_Number);

void LLD_TIMER_PIT_Init(PIT_Type *base, pit_chnl_t e_Channel);
static inline void LLD_TIMER_PIT_EnableInterrupts(PIT_Type *base, pit_chnl_t channel, uint32_t mask);
static inline void LLD_TIMER_PIT_DisableInterrupts(PIT_Type *base, pit_chnl_t channel, uint32_t mask);
static inline uint32_t LLD_TIMER_PIT_GetStatusFlags(PIT_Type *base, pit_chnl_t channel);
static inline void LLD_TIMER_PIT_ClearStatusFlags(PIT_Type *base, pit_chnl_t channel, uint32_t mask);
static inline void LLD_TIMER_PIT_SetTimerPeriod(PIT_Type *base, pit_chnl_t channel, uint32_t count);
static inline void LLD_TIMER_PIT_StartTimer(PIT_Type *base, pit_chnl_t channel);
static inline void LLD_TIMER_PIT_StopTimer(PIT_Type *base, pit_chnl_t channel);
static uint8_t LLD_TIMER_PIT_GetOffset(typ_Lld_Timer e_Number);
void PIT_DriverIRQHandler(void);
void LLD_TIMER_PIT_HandleIRQ(typ_Lld_Timer e_Number);

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*!
 * brief Initialize LLD TIMER GPT to reset state and initialize running mode.
 *
 * param base GPT peripheral base address.
 * param initConfig GPT mode setting configuration.
 */
void LLD_TIMER_GPT_Init(GPT_Type *base, const gpt_config_t *initConfig)
{
	assert(NULL != initConfig);
	if(base == GPT1)
    {
    	(void)CLOCK_EnableClock(kCLOCK_Gpt1);
    }
    else /* (base == GPT2) */
	{
		(void)CLOCK_EnableClock(kCLOCK_Gpt2);
	}

    base->CR = 0U;

    LLD_TIMER_GPT_SoftwareReset(base);

    base->CR =
        (initConfig->enableFreeRun ? GPT_CR_FRR_MASK : 0UL) | GPT_CR_WAITEN_MASK |
        GPT_CR_STOPEN_MASK | GPT_CR_ENMOD_MASK;

    base->CR |= (kGPT_ClockSource_Periph << GPT_CR_CLKSRC_SHIFT);

    /* Clock divider for peripheral clock of 60MHz */
    base->PR = 60UL - 1U;
}

/*!
 * @brief Software reset of GPT module.
 *
 * @param base GPT peripheral base address.
 */
static inline void LLD_TIMER_GPT_SoftwareReset(GPT_Type *base)
{
    base->CR |= GPT_CR_SWR_MASK;
    /* Wait reset finished. */
    while ((base->CR & GPT_CR_SWR_MASK) == GPT_CR_SWR_MASK)
    {
    }
}

/*!
 * @brief Start GPT timer.
 *
 * @param base GPT peripheral base address.
 */
static inline void LLD_TIMER_GPT_StartTimer(GPT_Type *base)
{
    base->CR |= GPT_CR_EN_MASK;
}

/*!
 * @brief Stop GPT timer.
 *
 * @param base GPT peripheral base address.
 */
static inline void LLD_TIMER_GPT_StopTimer(GPT_Type *base)
{
    base->CR &= ~GPT_CR_EN_MASK;
}

/*!
 * @brief Reads the current GPT counting value.
 *
 * @param base GPT peripheral base address.
 * @return Current GPT counter value.
 */
static inline uint32_t LLD_TIMER_GPT_GetCurrentTimerCount(GPT_Type *base)
{
    return base->CNT;
}

/*!
 * @brief Set GPT operation mode of input capture channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT capture channel (see @ref gpt_input_capture_channel_t typedef enumeration).
 * @param mode GPT input capture operation mode (see @ref gpt_input_operation_mode_t typedef enumeration).
 */
static inline void LLD_TIMER_GPT_SetInputOperationMode(GPT_Type *base,
														gpt_input_capture_channel_t channel,
														gpt_input_operation_mode_t mode)
{
    assert(channel <= kGPT_InputCapture_Channel2);

    base->CR =
        (base->CR & ~(GPT_CR_IM1_MASK << ((uint32_t)channel * 2UL))) | (GPT_CR_IM1(mode) << ((uint32_t)channel * 2UL));
}

/*!
 * @brief Set GPT operation mode of output compare channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 * @param mode GPT output operation mode (see @ref gpt_output_operation_mode_t typedef enumeration).
 */
static inline void LLD_TIMER_GPT_SetOutputOperationMode(GPT_Type *base,
                                              gpt_output_compare_channel_t channel,
                                              gpt_output_operation_mode_t mode)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    base->CR =
        (base->CR & ~(GPT_CR_OM1_MASK << ((uint32_t)channel * 3UL))) | (GPT_CR_OM1(mode) << ((uint32_t)channel * 3UL));
}

/*!
 * @brief Set GPT output compare value of output compare channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 * @param value GPT output compare value.
 */
static inline void LLD_TIMER_GPT_SetOutputCompareValue(GPT_Type *base, gpt_output_compare_channel_t channel, uint32_t value)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    base->OCR[(uint32_t)channel] = value;
}

/*!
 * @brief Enables the selected GPT interrupts.
 *
 * @param base GPT peripheral base address.
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::gpt_interrupt_enable_t
 */
static inline void LLD_TIMER_GPT_EnableInterrupts(GPT_Type *base, uint32_t mask)
{
    base->IR |= mask;
}

/*!
 * @brief Disables the selected GPT interrupts.
 *
 * @param base    GPT peripheral base address
 * @param mask    The interrupts to disable. This is a logical OR of members of the
 *                enumeration ::gpt_interrupt_enable_t
 */
static inline void LLD_TIMER_GPT_DisableInterrupts(GPT_Type *base, uint32_t mask)
{
    base->IR &= ~mask;
}

/*!
 * @brief Get GPT status flags.
 *
 * @param base GPT peripheral base address.
 * @param flags GPT status flag mask (see @ref gpt_status_flag_t for bit definition).
 * @return GPT status, each bit represents one status flag.
 */
static inline uint32_t LLD_TIMER_GPT_GetStatusFlags(GPT_Type *base, gpt_status_flag_t flags)
{
    return base->SR & (uint32_t)flags;
}

/*!
 * @brief Clears the GPT status flags.
 *
 * @param base GPT peripheral base address.
 * @param flags GPT status flag mask (see @ref gpt_status_flag_t for bit definition).
 */
void LLD_TIMER_GPT_ClearInputCaptureFlags(typ_Lld_Timer e_Number)
{
	uint8_t u8_TimerNumber;
	u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);
	LLD_TIMER_GPT_ClearStatusFlags(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, kGPT_InputCapture1Flag);
}

static inline void LLD_TIMER_GPT_ClearStatusFlags(GPT_Type *base, gpt_status_flag_t flags)
{
    base->SR = (uint32_t)flags;
}

/**
* @brief		Get GPT timer offset
* @param		e_Number GPT timer number
* @return		timer number
* @details
**/
static uint8_t LLD_TIMER_GPT_GetOffset(typ_Lld_Timer e_Number)
{
	uint8_t u8_Offset;
	if(LLD_TIMER_GPT1 == e_Number)
	{
		u8_Offset = 0;
	}
	else /* if(LLD_TIMER_GPT2 == e_Number) */
	{
		u8_Offset = 1;
	}

	return u8_Offset;
}

/*!
 * brief Ungates the PIT clock, enables the PIT module, and configures the peripheral for basic operations.
 *
 * note This API should be called at the beginning of the application using the PIT driver.
 *
 * param base   PIT peripheral base address
 */
void LLD_TIMER_PIT_Init(PIT_Type *base, pit_chnl_t e_Channel)
{
    /* Ungate the PIT clock*/
	CLOCK_EnableClock(kCLOCK_Pit);

    /* Enable PIT timers */
    base->MCR &= ~PIT_MCR_MDIS_MASK;

    /* Clear all status bits for channel to make sure the status of all TCTRL registers is clean. */
    base->CHANNEL[e_Channel].TCTRL &= ~(PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK | PIT_TCTRL_CHN_MASK);

    /* Timers are stopped in Debug mode */
    base->MCR |= PIT_MCR_FRZ_MASK;
}

/*!
 * @brief Enables the selected PIT interrupts.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number
 * @param mask    The interrupts to enable. This is a logical OR of members of the
 *                enumeration ::pit_interrupt_enable_t
 */
static inline void LLD_TIMER_PIT_EnableInterrupts(PIT_Type *base, pit_chnl_t channel, uint32_t mask)
{
    base->CHANNEL[channel].TCTRL |= mask;
}

/*!
 * @brief Disables the selected PIT interrupts.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number
 * @param mask    The interrupts to disable. This is a logical OR of members of the
 *                enumeration ::pit_interrupt_enable_t
 */
static inline void LLD_TIMER_PIT_DisableInterrupts(PIT_Type *base, pit_chnl_t channel, uint32_t mask)
{
    base->CHANNEL[channel].TCTRL &= ~mask;
}

/*!
 * @brief Gets the PIT status flags.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number
 *
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::pit_status_flags_t
 */
static inline uint32_t LLD_TIMER_PIT_GetStatusFlags(PIT_Type *base, pit_chnl_t channel)
{
    return (base->CHANNEL[channel].TFLG & PIT_TFLG_TIF_MASK);
}

/*!
 * @brief  Clears the PIT status flags.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number
 * @param mask    The status flags to clear. This is a logical OR of members of the
 *                enumeration ::pit_status_flags_t
 */
static inline void LLD_TIMER_PIT_ClearStatusFlags(PIT_Type *base, pit_chnl_t channel, uint32_t mask)
{
    base->CHANNEL[channel].TFLG = mask;
}

/*!
 * @brief Sets the timer period in units of count.
 *
 * Timers begin counting from the value set by this function until it reaches 0,
 * then it generates an interrupt and load this register value again.
 * Writing a new value to this register does not restart the timer. Instead, the value
 * is loaded after the timer expires.
 *
 * @note Users can call the utility macros provided in LLD_COMMON.h to convert to ticks.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number
 * @param count   Timer period in units of ticks
 */
static inline void LLD_TIMER_PIT_SetTimerPeriod(PIT_Type *base, pit_chnl_t channel, uint32_t count)
{
    assert(count != 0U);
    /* According to RM, the LDVAL trigger = clock ticks -1 */
    base->CHANNEL[channel].LDVAL = count - 1U;
}

/*!
 * @brief Starts the timer counting.
 *
 * After calling this function, timers load period value, count down to 0 and
 * then load the respective start value again. Each time a timer reaches 0,
 * it generates a trigger pulse and sets the timeout interrupt flag.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number.
 */
static inline void LLD_TIMER_PIT_StartTimer(PIT_Type *base, pit_chnl_t channel)
{
    base->CHANNEL[channel].TCTRL |= PIT_TCTRL_TEN_MASK;
}

/*!
 * @brief Stops the timer counting.
 *
 * This function stops every timer counting. Timers reload their periods
 * respectively after the next time they call the PIT_DRV_StartTimer.
 *
 * @param base    PIT peripheral base address
 * @param channel Timer channel number.
 */
static inline void LLD_TIMER_PIT_StopTimer(PIT_Type *base, pit_chnl_t channel)
{
    base->CHANNEL[channel].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}

/**
* @brief		Get PIT timer offset
* @param		e_Number PIT timer number
* @return		timer number
* @details
**/
static uint8_t LLD_TIMER_PIT_GetOffset(typ_Lld_Timer e_Number)
{
	uint8_t u8_Offset;
	if(LLD_TIMER_PIT0 == e_Number)
	{
		u8_Offset = 0;
	}
	else if(LLD_TIMER_PIT1 == e_Number)
	{
		u8_Offset = 1;
	}
	else if(LLD_TIMER_PIT2 == e_Number)
	{
		u8_Offset = 2;
	}
	else /* if(LLD_TIMER_PIT3 == e_Number) */
	{
		u8_Offset = 3;
	}

	return u8_Offset;
}


/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*!
 * brief LLD TIMER IRQ handle function.
 *
 * param e_Number GPT timer number
 */
void LLD_TIMER_GPT_HandleIRQ(typ_Lld_Timer e_Number)
{
	uint8_t u8_GptNumber;
	u8_GptNumber = LLD_TIMER_GPT_GetOffset(e_Number);

	if(LLD_TIMER_GPT_GetStatusFlags(ts_lld_timer_gpt_manager[u8_GptNumber].ps_Base, kGPT_OutputCompare1Flag) != 0)
	{
		/* Clear interrupt flag.*/
		LLD_TIMER_GPT_ClearStatusFlags(ts_lld_timer_gpt_manager[u8_GptNumber].ps_Base, kGPT_OutputCompare1Flag);

		/* Calling Callback Function if has one. */
		if (ts_lld_timer_gpt_manager[u8_GptNumber].s_ldd_timer_handle.callback != NULL)
		{
			ts_lld_timer_gpt_manager[u8_GptNumber].s_ldd_timer_handle.callback();
		}
	}
}

void GPT1_DriverIRQHandler(void)
{
	LLD_TIMER_GPT_HandleIRQ(LLD_TIMER_GPT1);
	SDK_ISR_EXIT_BARRIER;
}

void GPT2_DriverIRQHandler(void)
{
	LLD_TIMER_GPT_HandleIRQ(LLD_TIMER_GPT2);
	SDK_ISR_EXIT_BARRIER;
}

/**
* @brief		Read GPT timer value
* @param		e_Number GPT timer number
* @param		pu32_TimerValue GPT timer value
* @return		status : this function can't be used for PIT Timer
* @details
**/
typ_LldTimerStatus LLD_TIMER_GPT_Read(typ_Lld_Timer e_Number, uint32_t *pu32_TimerValue)
{
	typ_LldTimerStatus e_Result;

	if(e_Number <= LLD_TIMER_GPT2)
	{
		uint8_t u8_GptNumber = LLD_TIMER_GPT_GetOffset(e_Number);
		*pu32_TimerValue = LLD_TIMER_GPT_GetCurrentTimerCount(ts_lld_timer_gpt_manager[u8_GptNumber].ps_Base);
		e_Result = LLD_TIMER_STATUS_SUCCESS;
	}
	else
	{
		e_Result = LLD_TIMER_STATUS_ERROR;
	}

	return e_Result;
}

/**
* @brief		Set comparator value to trigger interruption
* @param		e_Number GPT timer number
* @param		u32_cmpValue value that trigger interruption
* @return		status : this function can't be used for PIT Timer
* @details
**/
typ_LldTimerStatus LLD_TIMER_GPT_SetComparator(typ_Lld_Timer e_Number, uint32_t u32_cmpValue)
{
	typ_LldTimerStatus e_Result;
	uint8_t u8_GptNumber;
	u8_GptNumber = LLD_TIMER_GPT_GetOffset(e_Number);

	if(e_Number <= LLD_TIMER_GPT2)
	{
		LLD_TIMER_GPT_SetOutputCompareValue(ts_lld_timer_gpt_manager[u8_GptNumber].ps_Base, kGPT_OutputCompare_Channel1, u32_cmpValue);
		e_Result = LLD_TIMER_STATUS_SUCCESS;
	}
	else
	{
		e_Result = LLD_TIMER_STATUS_ERROR;
	}

	return e_Result;
}

/**
* @brief		Set PIT timer initial value
* @param		e_Number PIT timer number
* @param		u32_initValue PIT timer initial value in microseconds
* @return		status : this function can't be used for GPT Timer
* @details
**/
typ_LldTimerStatus LLD_TIMER_PIT_SetTimerInitialValue(typ_Lld_Timer e_Number, uint32_t u32_initValue)
{
	typ_LldTimerStatus e_Result;
	uint8_t u8_PitNumber;
	u8_PitNumber = LLD_TIMER_PIT_GetOffset(e_Number);

	if(e_Number >= LLD_TIMER_PIT0)
	{
		LLD_TIMER_PIT_SetTimerPeriod(ts_lld_timer_pit_manager[u8_PitNumber].ps_Base, ts_lld_timer_pit_manager[u8_PitNumber].e_Channel, USEC_TO_COUNT(u32_initValue, PIT_SOURCE_CLOCK));
		e_Result = LLD_TIMER_STATUS_SUCCESS;
	}
	else
	{
		e_Result = LLD_TIMER_STATUS_ERROR;
	}

	return e_Result;
}

void LLD_TIMER_PIT0_HandleIRQ(void)
{
	uint8_t u8_PitNumber;
	u8_PitNumber = LLD_TIMER_PIT_GetOffset(LLD_TIMER_PIT0);

	/* Calling Callback Function if has one. */
	if (ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback != NULL)
	{
		ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback();
	}
}

void LLD_TIMER_PIT1_HandleIRQ(void)
{
	uint8_t u8_PitNumber;
	u8_PitNumber = LLD_TIMER_PIT_GetOffset(LLD_TIMER_PIT1);

	/* Calling Callback Function if has one. */
	if (ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback != NULL)
	{
		ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback();
	}
}

void LLD_TIMER_PIT2_HandleIRQ(void)
{
	uint8_t u8_PitNumber;
	u8_PitNumber = LLD_TIMER_PIT_GetOffset(LLD_TIMER_PIT2);

	/* Calling Callback Function if has one. */
	if (ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback != NULL)
	{
		ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback();
	}
}

void LLD_TIMER_PIT3_HandleIRQ(void)
{
	uint8_t u8_PitNumber;
	u8_PitNumber = LLD_TIMER_PIT_GetOffset(LLD_TIMER_PIT3);

	/* Calling Callback Function if has one. */
	if (ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback != NULL)
	{
		ts_lld_timer_pit_manager[u8_PitNumber].s_ldd_timer_handle.callback();
	}
}

void PIT_DriverIRQHandler(void)
{
	uint8_t u8_PitNum;

	for(u8_PitNum = LLD_TIMER_PIT0; u8_PitNum <= LLD_TIMER_PIT3; u8_PitNum++)
	{
		uint8_t u8_PitNumber;
		u8_PitNumber = LLD_TIMER_PIT_GetOffset(u8_PitNum);

		if(LLD_TIMER_PIT_GetStatusFlags(ts_lld_timer_pit_manager[u8_PitNumber].ps_Base, ts_lld_timer_pit_manager[u8_PitNumber].e_Channel) != 0 )
		{
			/* Clear interrupt flag.*/
			LLD_TIMER_PIT_ClearStatusFlags(ts_lld_timer_pit_manager[u8_PitNumber].ps_Base, ts_lld_timer_pit_manager[u8_PitNumber].e_Channel, kPIT_TimerFlag);

			if(u8_PitNum == LLD_TIMER_PIT0)
			{
				LLD_TIMER_PIT0_HandleIRQ();
			}
			else if(u8_PitNum == LLD_TIMER_PIT1)
			{
				LLD_TIMER_PIT1_HandleIRQ();
			}
			else if(u8_PitNum == LLD_TIMER_PIT2)
			{
				LLD_TIMER_PIT2_HandleIRQ();
			}
			else /* if(u8_PitNum == LLD_TIMER_PIT3) */
			{
				LLD_TIMER_PIT3_HandleIRQ();
			}
			u8_PitNum = LLD_TIMER_PIT3;
		}
	}

	SDK_ISR_EXIT_BARRIER;
}

/**
* @brief		TIMER peripheral initialization
* @param		e_Number : timer number
* @param		u8_EnableFreeRun mode : freeRun (1) or Restart (0) -> set but not used for PIT Timer
* @return
* @details
**/
void LLD_TIMER_InitGptInputCapture(typ_Lld_Timer e_Number, lld_timer_transfer_callback_t pf_callback)
{
	uint8_t u8_TimerNumber;
	if(e_Number <= LLD_TIMER_GPT2)
	{
		gpt_config_t mTimerConfigStruct;
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);

		if(e_Number == LLD_TIMER_GPT1)
		{
			ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base = GPT1;
			ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq = GPT1_IRQn;

		}
		else /* if(e_Number == LLD_TIMER_GTP2) */
		{
			ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base = GPT2;
			ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq = GPT2_IRQn;
		}

		mTimerConfigStruct.enableFreeRun = false;

		/* GPT device initialization */
		LLD_TIMER_GPT_Init(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, &mTimerConfigStruct);

	    /* Setup input capture on a gpt channel */
		LLD_TIMER_GPT_SetInputOperationMode(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, kGPT_InputCapture_Channel1, kGPT_InputOperation_RiseEdge);

		/* Enable interrupt */
		LLD_TIMER_GPT_EnableInterrupts(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, kGPT_InputCapture1InterruptEnable);

		/* Set Callback */
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);
		ts_lld_timer_gpt_manager[u8_TimerNumber].s_ldd_timer_handle.callback = pf_callback;

		/* Enable interrupt request in the NVIC. */
		EnableIRQ(ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq);

		/* Interrupt priority settings in the NVIC. */
		NVIC_SetPriority(ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq, 0);
	}
}

/**
* @brief		TIMER peripheral initialization
* @param		e_Number : timer number
* @param		u8_EnableFreeRun mode : freeRun (1) or Restart (0) -> set but not used for PIT Timer
* @return
* @details
**/
void LLD_TIMER_Init(typ_Lld_Timer e_Number, uint8_t u8_EnableFreeRun)
{
	uint8_t u8_TimerNumber;
	if(e_Number <= LLD_TIMER_GPT2)
	{
		gpt_config_t mTimerConfigStruct;
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);

		if(e_Number == LLD_TIMER_GPT1)
		{
			ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base = GPT1;
			ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq = GPT1_IRQn;

		}
		else /* if(e_Number == LLD_TIMER_GTP2) */
		{
			ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base = GPT2;
			ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq = GPT2_IRQn;
		}

		mTimerConfigStruct.enableFreeRun = u8_EnableFreeRun;

		/* GPT device initialization */
		LLD_TIMER_GPT_Init(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, &mTimerConfigStruct);

		/* Don't change output pin */
		LLD_TIMER_GPT_SetOutputOperationMode(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected);

		/* Interrupt priority settings in the NVIC. */
		NVIC_SetPriority(ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq, 0);
	}

	else /* if(e_Number >= LLD_TIMER_PIT0) */
	{
		u8_TimerNumber = LLD_TIMER_PIT_GetOffset(e_Number);

		ts_lld_timer_pit_manager[u8_TimerNumber].ps_Base = PIT;
		ts_lld_timer_pit_manager[u8_TimerNumber].e_Irq = PIT_IRQn;

		if(e_Number == LLD_TIMER_PIT0)
		{
			ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel = kPIT_Chnl_0;
		}

		else if(e_Number == LLD_TIMER_PIT1)
		{
			ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel = kPIT_Chnl_1;
		}

		else if(e_Number == LLD_TIMER_PIT2)
		{
			ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel = kPIT_Chnl_2;
		}

		else /* (e_Number == LLD_TIMER_PIT3) */
		{
			ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel = kPIT_Chnl_3;
		}

		/* Init pit module */
		LLD_TIMER_PIT_Init(ts_lld_timer_pit_manager[u8_TimerNumber].ps_Base, ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel);

		LLD_TIMER_PIT_SetTimerInitialValue(e_Number, 1);
	}
}

/**
* @brief		Enable/Disable Timer interrupt
* @param		e_Number : Timer number
* @param		u8_Enable : 0 = Disable | 1 = Enable
* @return		void
* @details
**/
void LLD_TIMER_EnableIrq(typ_Lld_Timer e_Number, uint8_t u8_Enable)
{
	uint8_t u8_TimerNumber;
	if(e_Number <= LLD_TIMER_GPT2)
	{
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);

		if(u8_Enable == 1)
		{
			/* Enable interrupt */
			LLD_TIMER_GPT_EnableInterrupts(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, kGPT_OutputCompare1InterruptEnable);

			/* Enable interrupt request in the NVIC. */
			EnableIRQ(ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq);
		}
		else
		{
			/* Disable interrupt request in the NVIC. */
			DisableIRQ(ts_lld_timer_gpt_manager[u8_TimerNumber].e_Irq);

			/* Disable interrupt */
			LLD_TIMER_GPT_DisableInterrupts(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base, kGPT_OutputCompare1InterruptEnable);
		}
	}
	else /* if(e_Number >= LLD_TIMER_PIT0) */
	{
		u8_TimerNumber = LLD_TIMER_PIT_GetOffset(e_Number);

		if(u8_Enable == 1)
		{
			/* Enable interrupt */
			LLD_TIMER_PIT_EnableInterrupts(ts_lld_timer_pit_manager[u8_TimerNumber].ps_Base, ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel, kPIT_TimerInterruptEnable);

			/* Enable interrupt request in the NVIC. */
			EnableIRQ(ts_lld_timer_pit_manager[u8_TimerNumber].e_Irq);
		}
		else
		{
			/* Disable interrupt request in the NVIC. */
			DisableIRQ(ts_lld_timer_pit_manager[u8_TimerNumber].e_Irq);

			/* Disable interrupt */
			LLD_TIMER_PIT_DisableInterrupts(ts_lld_timer_pit_manager[u8_TimerNumber].ps_Base, ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel, kPIT_TimerInterruptEnable);
		}
	}
}

/**
* @brief		Start timer
* @param 		e_Number : timer number
* @return		void
* @details
**/
void LLD_TIMER_Start(typ_Lld_Timer e_Number)
{
	uint8_t u8_TimerNumber;
	if(e_Number <= LLD_TIMER_GPT2)
	{
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);
		LLD_TIMER_GPT_StartTimer(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base);
	}
	else /* if(e_Number >= LLD_TIMER_PIT0) */
	{
		u8_TimerNumber = LLD_TIMER_PIT_GetOffset(e_Number);
		LLD_TIMER_PIT_StartTimer(ts_lld_timer_pit_manager[u8_TimerNumber].ps_Base, ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel);
	}
}

/**
* @brief		Stop timer
* @param		e_Number : timer number
* @return		void
* @details
**/
void LLD_TIMER_Stop(typ_Lld_Timer e_Number)
{
	uint8_t u8_TimerNumber;
	if(e_Number <= LLD_TIMER_GPT2)
	{
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);
		LLD_TIMER_GPT_StopTimer(ts_lld_timer_gpt_manager[u8_TimerNumber].ps_Base);
	}
	else /* if(e_Number >= LLD_TIMER_PIT0) */
	{
		u8_TimerNumber = LLD_TIMER_PIT_GetOffset(e_Number);
		LLD_TIMER_PIT_StopTimer(ts_lld_timer_pit_manager[u8_TimerNumber].ps_Base, ts_lld_timer_pit_manager[u8_TimerNumber].e_Channel);
	}
}

/**
* @brief		Set callback function
* @param		e_Number : timer number
* @param		pf_callback : Pointer on function
* @return		void
* @details
**/
void LLD_TIMER_SetCallback(typ_Lld_Timer e_Number, lld_timer_transfer_callback_t pf_callback)
{
	uint8_t u8_TimerNumber;
	if(e_Number <= LLD_TIMER_GPT2)
	{
		u8_TimerNumber = LLD_TIMER_GPT_GetOffset(e_Number);
		ts_lld_timer_gpt_manager[u8_TimerNumber].s_ldd_timer_handle.callback = pf_callback;
	}
	else /* if(e_Number >= LLD_TIMER_PIT0) */
	{
		u8_TimerNumber = LLD_TIMER_PIT_GetOffset(e_Number);
		ts_lld_timer_pit_manager[u8_TimerNumber].s_ldd_timer_handle.callback = pf_callback;
	}
}
