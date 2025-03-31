/**
 * @file LLD_PWM.c
 * @author ACR
 * @brief Specific PWM driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_PWM_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "MIMXRT1062.h"
#include "MIMXRT1062_features.h"
#include "fsl_clock.h"
#include "LLD_PWM.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES PWM ...                                                */
/*--------------------------------------------------------------------------*/

/*! @brief List of PWM submodules */
typedef enum _pwm_submodule
{
    kPWM_Module_0 = 0U, /*!< Submodule 0 */
    kPWM_Module_1,      /*!< Submodule 1 */
    kPWM_Module_2,      /*!< Submodule 2 */
    kPWM_Module_3       /*!< Submodule 3 */
} pwm_submodule_t;

/*! @brief List of PWM channels in each module */
typedef enum _pwm_channels
{
    kPWM_PwmB = 0U,
    kPWM_PwmA,
    kPWM_PwmX
} pwm_channels_t;

/*! @brief PWM prescaler factor selection for clock source*/
typedef enum _pwm_clock_prescale
{
    kPWM_Prescale_Divide_1 = 0U, /*!< PWM clock frequency = fclk/1 */
    kPWM_Prescale_Divide_2,      /*!< PWM clock frequency = fclk/2 */
    kPWM_Prescale_Divide_4,      /*!< PWM clock frequency = fclk/4 */
    kPWM_Prescale_Divide_8,      /*!< PWM clock frequency = fclk/8 */
    kPWM_Prescale_Divide_16,     /*!< PWM clock frequency = fclk/16 */
    kPWM_Prescale_Divide_32,     /*!< PWM clock frequency = fclk/32 */
    kPWM_Prescale_Divide_64,     /*!< PWM clock frequency = fclk/64 */
    kPWM_Prescale_Divide_128     /*!< PWM clock frequency = fclk/128 */
} pwm_clock_prescale_t;

/*! @brief List of PWM fault selections */
typedef enum _pwm_fault_input
{
    kPWM_Fault_0 = 0U, /*!< Fault 0 input pin */
    kPWM_Fault_1,      /*!< Fault 1 input pin */
    kPWM_Fault_2,      /*!< Fault 2 input pin */
    kPWM_Fault_3       /*!< Fault 3 input pin */
} pwm_fault_input_t;

/*! @brief List of PWM fault disable mapping selections */
typedef enum _pwm_fault_disable
{
    kPWM_FaultDisable_0 = (1U << 0), /*!< Fault 0 disable mapping */
    kPWM_FaultDisable_1 = (1U << 1), /*!< Fault 1 disable mapping */
    kPWM_FaultDisable_2 = (1U << 2), /*!< Fault 2 disable mapping */
    kPWM_FaultDisable_3 = (1U << 3)  /*!< Fault 3 disable mapping */
} pwm_fault_disable_t;

/*! @brief List of PWM fault channels */
typedef enum _pwm_fault_channels
{
    kPWM_faultchannel_0 = 0U,
    kPWM_faultchannel_1
} pwm_fault_channels_t;

/*! @brief Options for submodule master control operation */
typedef enum _pwm_module_control
{
    kPWM_Control_Module_0 = (1U << 0), /*!< Control submodule 0's start/stop,buffer reload operation */
    kPWM_Control_Module_1 = (1U << 1), /*!< Control submodule 1's start/stop,buffer reload operation */
    kPWM_Control_Module_2 = (1U << 2), /*!< Control submodule 2's start/stop,buffer reload operation */
    kPWM_Control_Module_3 = (1U << 3)  /*!< Control submodule 3's start/stop,buffer reload operation */
} pwm_module_control_t;

/*! @brief PWM output fault status */
typedef enum _pwm_fault_state
{
    kPWM_PwmFaultState0 =
        0U,              /*!< Output is forced to logic 0 state prior to consideration of output polarity control. */
    kPWM_PwmFaultState1, /*!< Output is forced to logic 1 state prior to consideration of output polarity control. */
    kPWM_PwmFaultState2, /*!< Output is tristated. */
    kPWM_PwmFaultState3  /*!< Output is tristated. */
} pwm_fault_state_t;

/*! @brief Structure for the user to define the PWM signal characteristics */
typedef struct _pwm_signal_param
{
    uint8_t dutyCyclePercent;  /*!< PWM pulse width, value should be between 0 to 100
                                    0=inactive signal(0% duty cycle)...
                                    100=always active signal (100% duty cycle)*/
    pwm_channels_t pwmChannel; /*!< PWM channel being configured; PWM A or PWM B */
    pwm_fault_state_t faultState; /*!< PWM output fault status */
} pwm_signal_param_t;

/*!
 * @brief PWM config structure
 */
typedef struct _pwm_config
{
    pwm_clock_prescale_t prescale;           /*!< Pre-scaler to divide down the clock */
} pwm_config_t;

/*--------------------------------------------------------------------------*/
/* ... DATATYPES XBARA ...                                                  */
/*--------------------------------------------------------------------------*/

/* Macros for entire XBARA_CTRL register.  */
#define XBARA_CTRLx(base, index) (((volatile uint16_t *)(&((base)->CTRL0)))[(index)])

typedef union
{
    uint8_t _u8[2];
    uint16_t _u16;
} xbara_u8_u16_t;

/* Macros for entire XBARA_SELx register.  */
#define XBARA_SELx(base, output) (((volatile uint16_t *)(&((base)->SEL0)))[(uint32_t)(output) / 2UL])

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD PWM ...                                                */
/*--------------------------------------------------------------------------*/

typedef struct
{
	PWM_Type * ps_Port;				/* Instance : PWM1 to PWM4 */
	pwm_submodule_t e_submodule;	/* PWM Submodule : 0 to 3 */
	uint8_t u8_dutyCycle_A;			/* PWM pulse width of channel A in percent : 0 to 100 */
	uint8_t u8_dutyCycle_B;			/* PWM pulse width of channel B in percent : 0 to 100 */
	uint32_t u32_frequency;			/* PWM signal frequency : 100Hz to 1MHz */
}typ_Lld_Pwm_manager;

typ_Lld_Pwm_manager ts_lld_pwm_manager[LLD_PWM_NB];

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

static inline uint16_t LLD_PWM_GetComplementU16(uint16_t value);
static inline void LLD_PWM_StartTimer(PWM_Type *base, uint8_t subModulesToStart);
static inline void LLD_PWM_StopTimer(PWM_Type *base, uint8_t subModulesToStop);
static inline void LLD_PWM_SetPwmLdok(PWM_Type *base, uint8_t subModulesToUpdate, bool value);
static inline void LLD_PWM_SetupFaultDisableMap(PWM_Type *base, pwm_submodule_t subModule,
                                            pwm_fault_channels_t pwm_fault_channels, uint16_t value);
static inline uint16_t LLD_LLD_PWM_GetComplementU16(uint16_t value);
static inline uint16_t LLD_PWM_DutyCycleToReloadValue(uint8_t dutyCyclePercent);
status_t PWM_Init(PWM_Type *base, pwm_submodule_t subModule, const pwm_config_t *config);
status_t LLD_PWM_SetupPwm(PWM_Type *base, pwm_submodule_t subModule, const pwm_signal_param_t *chnlParams,
                      uint8_t numOfChnls, uint32_t pwmFreq_Hz, uint32_t srcClock_Hz);
void LLD_PWM_UpdatePwmDutycycle(PWM_Type *base, pwm_submodule_t subModule, pwm_channels_t pwmSignal, uint8_t dutyCyclePercent);
void LLD_PWM_UpdatePwmDutycycleHighAccuracy(PWM_Type *base, pwm_submodule_t subModule, pwm_channels_t pwmSignal, uint16_t dutyCycle);
void LLD_PWM_SetupFaults(PWM_Type *base, pwm_fault_input_t faultNum);

void LLD_PWM_XBARA_Init(XBARA_Type *base);
void LLD_PWM_XBARA_SetSignalsConnection(XBARA_Type *base, xbar_input_signal_t input, xbar_output_signal_t output);

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*!
 * brief Complement the variable of type uint16_t as needed
 *
 * This function can complement the variable of type uint16_t as needed.For example,
 * need to ask for the opposite of a positive integer.
 *
 * param value    Parameters of type uint16_t
 */
static inline uint16_t LLD_PWM_GetComplementU16(uint16_t value)
{
    return (~value + 1U);
}

/*!
 * @brief Starts the PWM counter for a single or multiple submodules.
 *
 * Sets the Run bit which enables the clocks to the PWM submodule. This function can start multiple
 * submodules at the same time.
 *
 * @param base              PWM peripheral base address
 * @param subModulesToStart PWM submodules to start. This is a logical OR of members of the
 *                          enumeration ::pwm_module_control_t
 */
static inline void LLD_PWM_StartTimer(PWM_Type *base, uint8_t subModulesToStart)
{
    base->MCTRL |= PWM_MCTRL_RUN(subModulesToStart);
}

/*!
 * @brief Stops the PWM counter for a single or multiple submodules.
 *
 * Clears the Run bit which resets the submodule's counter. This function can stop multiple
 * submodules at the same time.
 *
 * @param base             PWM peripheral base address
 * @param subModulesToStop PWM submodules to stop. This is a logical OR of members of the
 *                         enumeration ::pwm_module_control_t
 */
static inline void LLD_PWM_StopTimer(PWM_Type *base, uint8_t subModulesToStop)
{
    base->MCTRL &= ~(PWM_MCTRL_RUN(subModulesToStop));
}

/*!
 * @brief Sets or clears the PWM LDOK bit on a single or multiple submodules
 *
 * Set LDOK bit to load buffered values into CTRL[PRSC] and the INIT, FRACVAL and VAL registers. The
 * values are loaded immediately if kPWM_ReloadImmediate option was choosen during config. Else the
 * values are loaded at the next PWM reload point.
 * This function can issue the load command to multiple submodules at the same time.
 *
 * @param base               PWM peripheral base address
 * @param subModulesToUpdate PWM submodules to update with buffered values. This is a logical OR of
 *                           members of the enumeration ::pwm_module_control_t
 * @param value              true: Set LDOK bit for the submodule list; false: Clear LDOK bit
 */
static inline void LLD_PWM_SetPwmLdok(PWM_Type *base, uint8_t subModulesToUpdate, bool value)
{
    if (value)
    {
        base->MCTRL |= PWM_MCTRL_LDOK(subModulesToUpdate);
    }
    else
    {
        base->MCTRL |= PWM_MCTRL_CLDOK(subModulesToUpdate);
    }
}

/*!
 * @brief Set PWM fault disable mapping
 *
 * Each of the four bits of this read/write field is one-to-one associated
 * with the four FAULTx inputs of fault channel 0/1. The PWM output will be turned
 * off if there is a logic 1 on an FAULTx input and a 1 in the corresponding
 * bit of this field. A reset sets all bits in this field.
 *
 * @param base               PWM peripheral base address
 * @param subModule          PWM submodule to configure
 * @param pwm_fault_channels PWM fault channel to configure
 * @param value              Fault disable mapping mask value
 *                           enumeration ::pwm_fault_disable_t
 */
static inline void LLD_PWM_SetupFaultDisableMap(PWM_Type *base, pwm_submodule_t subModule, pwm_fault_channels_t pwm_fault_channels, uint16_t value)
{
    uint16_t reg = base->SM[subModule].DISMAP[pwm_fault_channels];

    /* Use channel A only */
	reg &= ~((uint16_t)PWM_DISMAP_DIS0A_MASK);
	reg |= (((uint16_t)(value) << (uint16_t)PWM_DISMAP_DIS0A_SHIFT) & (uint16_t)PWM_DISMAP_DIS0A_MASK);

    base->SM[subModule].DISMAP[pwm_fault_channels] = reg;
}

/*!
 * brief Complement the variable of type uint16_t as needed
 *
 * This function can complement the variable of type uint16_t as needed.For example,
 * need to ask for the opposite of a positive integer.
 *
 * param value    Parameters of type uint16_t
 */
static inline uint16_t LLD_LLD_PWM_GetComplementU16(uint16_t value)
{
    return (~value + 1U);
}

static inline uint16_t LLD_PWM_DutyCycleToReloadValue(uint8_t dutyCyclePercent)
{
    /* Rounding calculations to improve the accuracy of reloadValue */
    return ((65535U * dutyCyclePercent) + 50U) / 100U;
}

/*!
 * brief Ungates the PWM submodule clock and configures the peripheral for basic operation.
 *
 * note This API should be called at the beginning of the application using the PWM driver.
 *
 * param base      PWM peripheral base address
 * param subModule PWM submodule to configure
 * param config    Pointer to user's PWM config structure.
 *
 * return LLD_STATUS_Success means success; else failed.
 */
status_t PWM_Init(PWM_Type *base, pwm_submodule_t subModule, const pwm_config_t *config)
{
    assert(config);

    uint16_t reg;

    /* Ungate the PWM submodule clock*/
    if(base == PWM1)
    {
    	CLOCK_EnableClock(kCLOCK_Pwm1);
    }
    else if(base == PWM2)
	{
    	CLOCK_EnableClock(kCLOCK_Pwm2);
	}
    else if(base == PWM3)
	{
    	CLOCK_EnableClock(kCLOCK_Pwm3);
	}
    else /* if(base == PWM4) */
	{
    	CLOCK_EnableClock(kCLOCK_Pwm4);
	}

    /* Clear the fault status flags */
    base->FSTS |= PWM_FSTS_FFLAG_MASK;

    reg = base->SM[subModule].CTRL2;

    /* Setup PWM A & B to be independent */
    reg |= PWM_CTRL2_INDEP_MASK;

    /* Issue a Force trigger event when configured to trigger locally */
    reg |= PWM_CTRL2_FORCE(1U);

    base->SM[subModule].CTRL2 = reg;

    reg = base->SM[subModule].CTRL;

    /* Setup the clock prescale, load mode and frequency */
    reg &= ~(uint16_t)(PWM_CTRL_PRSC_MASK);
    reg |= PWM_CTRL_PRSC(config->prescale);

    /* Buffered-registers get loaded with new values as soon as LDOK bit is set */
    reg |= PWM_CTRL_LDMOD_MASK;

    base->SM[subModule].CTRL = reg;

    return kStatus_Success;
}

/*!
 * brief Sets up the PWM signals for a PWM submodule.
 *
 * The function initializes the submodule according to the parameters passed in by the user. The function
 * also sets up the value compare registers to match the PWM signal requirements.
 * If the dead time insertion logic is enabled, the pulse period is reduced by the
 * dead time period specified by the user.
 *
 * param base        PWM peripheral base address
 * param subModule   PWM submodule to configure
 * param chnlParams  Array of PWM channel parameters to configure the channel(s)
 * param numOfChnls  Number of channels to configure, this should be the size of the array passed in.
 *                    Array size should not be more than 2 as each submodule has 2 pins to output PWM
 * param pwmFreq_Hz  PWM signal frequency in Hz
 * param srcClock_Hz PWM main counter clock in Hz.
 *
 * return Returns kStatusFail if there was error setting up the signal; kStatusSuccess otherwise
 */
//status_t LLD_PWM_SetupPwm(PWM_Type *base, pwm_submodule_t subModule, const pwm_signal_param_t *chnlParams, uint8_t numOfChnls,
//							uint32_t pwmFreq_Hz, uint32_t srcClock_Hz)
//{
//    assert(chnlParams);
//    assert(pwmFreq_Hz);
//    assert(numOfChnls);
//    assert(srcClock_Hz);
//
//    uint32_t pwmClock;
//    uint16_t pulseCnt = 0, pwmHighPulse = 0;
//    uint16_t modulo = 0;
//    uint8_t i, polarityShift = 0, outputEnableShift = 0;
//
//    if (numOfChnls > 2U)
//    {
//        return kStatus_Fail;
//    }
//
//    /* Divide the clock by the prescale value */
//    pwmClock = (srcClock_Hz / (1UL << ((base->SM[subModule].CTRL & PWM_CTRL_PRSC_MASK) >> PWM_CTRL_PRSC_SHIFT)));
//    pulseCnt = (uint16_t)(pwmClock / pwmFreq_Hz);
//
//    /* Setup each PWM channel */
//    for (i = 0; i < numOfChnls; i++)
//    {
//        /* Calculate pulse width */
//        pwmHighPulse = (pulseCnt * chnlParams->dutyCyclePercent) / 100U;
//
//		/* Setup the PWM period for a signed center aligned signal */
//		if (i == 0U)
//		{
//			modulo = (pulseCnt >> 1U);
//			/* Indicates the start of the PWM period */
//			base->SM[subModule].INIT = LLD_LLD_PWM_GetComplementU16(modulo);
//			/* Indicates the center value */
//			base->SM[subModule].VAL0 = 0;
//			/* Indicates the end of the PWM period */
//			/* The change during the end to start of the PWM period requires a count time */
//			base->SM[subModule].VAL1 = modulo - 1U;
//		}
//
//		/* Setup the PWM dutycycle on channel A */
//		base->SM[subModule].VAL2 = LLD_LLD_PWM_GetComplementU16(pwmHighPulse / 2U);
//		base->SM[subModule].VAL3 = (pwmHighPulse / 2U);
//
//        /* Setup register shift values based on channel A */
//		polarityShift              = PWM_OCTRL_POLA_SHIFT;
//		outputEnableShift          = PWM_OUTEN_PWMA_EN_SHIFT;
//
//        /* Setup signal active high level */
//        base->SM[subModule].OCTRL &= ~((uint16_t)1U << (uint16_t)polarityShift);
//        /* Enable PWM output */
//        base->OUTEN |= ((uint16_t)1U << ((uint16_t)outputEnableShift + (uint16_t)subModule));
//
//        /* Get the next channel parameters */
//        chnlParams++;
//    }
//
//    return kStatus_Success;
//}

status_t LLD_PWM_SetupPwm(PWM_Type *base, pwm_submodule_t subModule, const pwm_signal_param_t *chnlParams, uint8_t numOfChnls,
							uint32_t pwmFreq_Hz, uint32_t srcClock_Hz)
{
    assert(chnlParams);
    assert(pwmFreq_Hz);
    assert(numOfChnls);
    assert(srcClock_Hz);

    uint32_t pwmClock;
    uint16_t pulseCnt = 0, pwmHighPulse = 0;
    uint16_t modulo = 0;
    uint8_t i, polarityShift = 0, outputEnableShift = 0;

    if (numOfChnls > 2U)
    {
        /* Each submodule has 2 signals; PWM A & PWM B */
        return kStatus_Fail;
    }

    /* Divide the clock by the prescale value */
    pwmClock = (srcClock_Hz / (1UL << ((base->SM[subModule].CTRL & PWM_CTRL_PRSC_MASK) >> PWM_CTRL_PRSC_SHIFT)));
    pulseCnt = (uint16_t)(pwmClock / pwmFreq_Hz);

    /* Setup each PWM channel */
    for (i = 0; i < numOfChnls; i++)
    {
        /* Calculate pulse width */
        pwmHighPulse = (pulseCnt * chnlParams->dutyCyclePercent) / 100U;

		/* Setup the PWM period for a signed center aligned signal */
		if (i == 0U)
		{
			modulo = (pulseCnt >> 1U);
			/* Indicates the start of the PWM period */
			base->SM[subModule].INIT = LLD_PWM_GetComplementU16(modulo);
			/* Indicates the center value */
			base->SM[subModule].VAL0 = 0;
			/* Indicates the end of the PWM period */
			/* The change during the end to start of the PWM period requires a count time */
			base->SM[subModule].VAL1 = modulo - 1U;
		}

		/* Setup the PWM dutycycle */
		if (chnlParams->pwmChannel == kPWM_PwmA)
		{
			base->SM[subModule].VAL2 = LLD_PWM_GetComplementU16(pwmHighPulse / 2U);
			base->SM[subModule].VAL3 = (pwmHighPulse / 2U);
		}
		else
		{
			base->SM[subModule].VAL4 = LLD_PWM_GetComplementU16(pwmHighPulse / 2U);
			base->SM[subModule].VAL5 = (pwmHighPulse / 2U);
		}

        /* Setup register shift values based on the channel being configured.
         * Also setup the deadtime value
         */
        if (chnlParams->pwmChannel == kPWM_PwmA)
        {
            polarityShift              = PWM_OCTRL_POLA_SHIFT;
            outputEnableShift          = PWM_OUTEN_PWMA_EN_SHIFT;
        }
        else
        {
            polarityShift              = PWM_OCTRL_POLB_SHIFT;
            outputEnableShift          = PWM_OUTEN_PWMB_EN_SHIFT;
        }

        /* Set PWM output fault status */
        switch (chnlParams->pwmChannel)
        {
            case kPWM_PwmA:
                base->SM[subModule].OCTRL &= ~((uint16_t)PWM_OCTRL_PWMAFS_MASK);
                base->SM[subModule].OCTRL |= (((uint16_t)(chnlParams->faultState) << (uint16_t)PWM_OCTRL_PWMAFS_SHIFT) &
                                              (uint16_t)PWM_OCTRL_PWMAFS_MASK);
                break;
            case kPWM_PwmB:
                base->SM[subModule].OCTRL &= ~((uint16_t)PWM_OCTRL_PWMBFS_MASK);
                base->SM[subModule].OCTRL |= (((uint16_t)(chnlParams->faultState) << (uint16_t)PWM_OCTRL_PWMBFS_SHIFT) &
                                              (uint16_t)PWM_OCTRL_PWMBFS_MASK);
                break;
            case kPWM_PwmX:
                base->SM[subModule].OCTRL &= ~((uint16_t)PWM_OCTRL_PWMXFS_MASK);
                base->SM[subModule].OCTRL |= (((uint16_t)(chnlParams->faultState) << (uint16_t)PWM_OCTRL_PWMXFS_SHIFT) &
                                              (uint16_t)PWM_OCTRL_PWMXFS_MASK);
                break;
            default:
                assert(false);
                break;
        }

        /* Setup signal active level */
        base->SM[subModule].OCTRL &= ~((uint16_t)1U << (uint16_t)polarityShift);
        /* Enable PWM output */
        base->OUTEN |= ((uint16_t)1U << ((uint16_t)outputEnableShift + (uint16_t)subModule));

        /* Get the next channel parameters */
        chnlParams++;
    }

    return kStatus_Success;
}

/*!
 * brief Updates the PWM signal's dutycycle.
 *
 * The function updates the PWM dutycyle to the new value that is passed in.
 * If the dead time insertion logic is enabled then the pulse period is reduced by the
 * dead time period specified by the user.
 *
 * param base              PWM peripheral base address
 * param subModule         PWM submodule to configure
 * param pwmSignal         Signal (PWM A or PWM B) to update
 * param dutyCyclePercent  New PWM pulse width, value should be between 0 to 100
 *                          0=inactive signal(0% duty cycle)...
 *                          100=active signal (100% duty cycle)
 */
void LLD_PWM_UpdatePwmDutycycle(PWM_Type *base, pwm_submodule_t subModule, pwm_channels_t pwmSignal, uint8_t dutyCyclePercent)
{
    assert(dutyCyclePercent <= 100U);
    assert((uint16_t)pwmSignal < 2U);
    uint16_t reloadValue = LLD_PWM_DutyCycleToReloadValue(dutyCyclePercent);

    LLD_PWM_UpdatePwmDutycycleHighAccuracy(base, subModule, pwmSignal, reloadValue);
}

/*!
 * brief Updates the PWM signal's dutycycle with 16-bit accuracy.
 *
 * The function updates the PWM dutycyle to the new value that is passed in.
 * If the dead time insertion logic is enabled then the pulse period is reduced by the
 * dead time period specified by the user.
 *
 * param base              PWM peripheral base address
 * param subModule         PWM submodule to configure
 * param pwmSignal         Signal (PWM A or PWM B) to update
 * param dutyCycle         New PWM pulse width, value should be between 0 to 65535
 *                          0=inactive signal(0% duty cycle)...
 *                          65535=active signal (100% duty cycle)
 */
void LLD_PWM_UpdatePwmDutycycleHighAccuracy(PWM_Type *base, pwm_submodule_t subModule, pwm_channels_t pwmSignal, uint16_t dutyCycle)
{
    assert((uint16_t)pwmSignal < 2U);
    uint16_t pulseCnt = 0, pwmHighPulse = 0;
    uint16_t modulo = 0;

    modulo   = base->SM[subModule].VAL1 + 1U;
	pulseCnt = modulo * 2U;
	/* Calculate pulse width */
	pwmHighPulse = (pulseCnt * dutyCycle) / 65535U;

	/* Setup the PWM dutycycle */
	if (pwmSignal == kPWM_PwmA)
	{
		base->SM[subModule].VAL2 = LLD_PWM_GetComplementU16(pwmHighPulse / 2U);
		base->SM[subModule].VAL3 = (pwmHighPulse / 2U);
	}
	else
	{
		base->SM[subModule].VAL4 = LLD_PWM_GetComplementU16(pwmHighPulse / 2U);
		base->SM[subModule].VAL5 = (pwmHighPulse / 2U);
	}
}

/*!
 * brief Sets up the PWM fault protection.
 *
 * PWM has 4 fault inputs.
 *
 * param base        PWM peripheral base address
 * param faultNum    PWM fault to configure.
 */
void LLD_PWM_SetupFaults(PWM_Type *base, pwm_fault_input_t faultNum)
{
    uint16_t reg;

    reg = base->FCTRL;

    /* Set the faults level to 0 */
    reg &= ~((uint16_t)1U << (PWM_FCTRL_FLVL_SHIFT + (uint16_t)faultNum));

	/* Use automatic fault clearing */
	reg |= ((uint16_t)1U << (PWM_FCTRL_FAUTO_SHIFT + (uint16_t)faultNum));
    base->FCTRL = reg;

	/* Combinational path from the fault input to the PWM output is available */
	base->FCTRL2 &= ~((uint16_t)1U << (uint16_t)faultNum);

    /* Initially clear both recovery modes */
    reg = base->FSTS;
    reg &= ~(((uint16_t)1U << (PWM_FSTS_FFULL_SHIFT + (uint16_t)faultNum)) |
             ((uint16_t)1U << (PWM_FSTS_FHALF_SHIFT + (uint16_t)faultNum)));

    base->FSTS = reg;
}

/*!
 * brief Initializes the XBARA module.
 *
 * This function un-gates the XBARA clock.
 *
 * param base XBARA peripheral address.
 */
void LLD_PWM_XBARA_Init(XBARA_Type *base)
{
    /* Enable XBARA module clock. */
    CLOCK_EnableClock(kCLOCK_Xbar1);
}

/*!
 * brief Sets a connection between the selected XBARA_IN[*] input and the XBARA_OUT[*] output signal.
 *
 * This function connects the XBARA input to the selected XBARA output.
 * If more than one XBARA module is available, only the inputs and outputs from the same module
 * can be connected.
 *
 * param base XBARA peripheral address.
 * param input XBARA input signal.
 * param output XBARA output signal.
 */
void LLD_PWM_XBARA_SetSignalsConnection(XBARA_Type *base, xbar_input_signal_t input, xbar_output_signal_t output)
{
    xbara_u8_u16_t regVal;
    uint8_t byteInReg;
    uint8_t outputIndex = (uint8_t)output;

    byteInReg = outputIndex % 2U;

    regVal._u16 = XBARA_SELx(base, outputIndex);

    regVal._u8[byteInReg] = (uint8_t)input;

    XBARA_SELx(base, outputIndex) = regVal._u16;
}

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		Pulse Width Modulation initialization
* @param		e_Pwm : PWM number
* @param		e_Channel : Channel A or B
* @param		u32_Frequency : frequency to configure
* @return		void
* @details
**/
void LLD_PWM_Init(typ_Lld_Pwm e_Pwm, typ_Lld_Pwm_Channel e_Channel, uint32_t u32_Frequency)
{
	pwm_config_t mod_config;
	pwm_signal_param_t modSignal[1];


	/* Set the PWM Fault inputs to a low value */
	LLD_PWM_XBARA_Init(XBARA1);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault0);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault1);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm2Fault0);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm2Fault1);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm3Fault0);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm3Fault1);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm4Fault0);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm4Fault1);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1234Fault2);
	LLD_PWM_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1234Fault3);

	/* Sets up the PWM fault protection */
	LLD_PWM_SetupFaults(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Fault_0);
	LLD_PWM_SetupFaults(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Fault_1);
	LLD_PWM_SetupFaults(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Fault_2);
	LLD_PWM_SetupFaults(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Fault_3);
	LLD_PWM_SetupFaultDisableMap(ts_lld_pwm_manager[e_Pwm].ps_Port, ts_lld_pwm_manager[e_Pwm].e_submodule,
			kPWM_faultchannel_0, kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3);

	/* PWM initialization */
	if(e_Pwm == LLD_PWM_PWM1_MODULE0)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM1;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_0;
	}
	else if(e_Pwm == LLD_PWM_PWM1_MODULE1)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM1;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_1;
	}
	else if(e_Pwm == LLD_PWM_PWM1_MODULE2)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM1;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_2;
	}
	else if(e_Pwm == LLD_PWM_PWM1_MODULE3)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM1;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_3;
	}
	else if(e_Pwm == LLD_PWM_PWM2_MODULE0)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM2;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_0;
	}
	else if(e_Pwm == LLD_PWM_PWM2_MODULE1)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM2;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_1;
	}
	else if(e_Pwm == LLD_PWM_PWM2_MODULE2)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM2;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_2;
	}
	else if(e_Pwm == LLD_PWM_PWM2_MODULE3)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM2;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_3;
	}
	else if(e_Pwm == LLD_PWM_PWM3_MODULE0)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM3;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_0;
	}
	else if(e_Pwm == LLD_PWM_PWM3_MODULE1)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM3;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_1;
	}
	else if(e_Pwm == LLD_PWM_PWM3_MODULE2)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM3;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_2;
	}
	else if(e_Pwm == LLD_PWM_PWM3_MODULE3)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM3;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_3;
	}
	else if(e_Pwm == LLD_PWM_PWM4_MODULE0)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM4;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_0;
	}
	else if(e_Pwm == LLD_PWM_PWM4_MODULE1)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM4;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_1;
	}
	else if(e_Pwm == LLD_PWM_PWM4_MODULE2)
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM4;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_2;
	}
	else /* if(e_Pwm == LLD_PWM_PWM4_MODULE3) */
	{
		ts_lld_pwm_manager[e_Pwm].ps_Port = PWM4;
		ts_lld_pwm_manager[e_Pwm].e_submodule = kPWM_Module_3;
	}

	/* Sets frequency */
	ts_lld_pwm_manager[e_Pwm].u32_frequency = u32_Frequency;

	if(ts_lld_pwm_manager[e_Pwm].u32_frequency < 10000)
	{
		mod_config.prescale = kPWM_Prescale_Divide_128;
	}
	else
	{
		mod_config.prescale = kPWM_Prescale_Divide_1;
	}

	/* Initialization of duty cycle value */
	if(e_Channel == LLD_PWM_CHANNEL_A)
	{
		ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_A = 0;
	}
	else /* if(e_Channel == LLD_PWM_CHANNEL_B) */
	{
		ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_B = 0;
	}

	PWM_Init(ts_lld_pwm_manager[e_Pwm].ps_Port, ts_lld_pwm_manager[e_Pwm].e_submodule, &mod_config);

	/* Channel configuration */
	modSignal[0].pwmChannel = e_Channel;
	modSignal[0].dutyCyclePercent = 0;
	LLD_PWM_SetupPwm(ts_lld_pwm_manager[e_Pwm].ps_Port, ts_lld_pwm_manager[e_Pwm].e_submodule, modSignal, 1,
			ts_lld_pwm_manager[e_Pwm].u32_frequency, SOURCE_CLK);
}

/**
* @brief		Enable/Disable the PWM
* @param[in]	e_Pwm : PWM to start
* @param		e_Channel : Channel A or B
* @param		Enable : activate or not PWM
* @return		void
* @details
**/
void LLD_PWM_Enable(typ_Lld_Pwm e_Pwm, typ_Lld_Pwm_Channel e_Channel, bool Enable)
{
	if(Enable == true)
	{
		if((ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_A != 0) || (ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_B != 0))
		{
			/* Update duty cycle */
			if(e_Channel == LLD_PWM_CHANNEL_A)
			{
				LLD_PWM_UpdatePwmDutycycle(ts_lld_pwm_manager[e_Pwm].ps_Port, ts_lld_pwm_manager[e_Pwm].e_submodule, e_Channel, ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_A);
			}
			else /* if(e_Channel == LLD_PWM_CHANNEL_B) */
			{
				LLD_PWM_UpdatePwmDutycycle(ts_lld_pwm_manager[e_Pwm].ps_Port, ts_lld_pwm_manager[e_Pwm].e_submodule, e_Channel, ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_B);
			}

			/* Start */
			if(ts_lld_pwm_manager[e_Pwm].e_submodule == kPWM_Module_0)
			{
				LLD_PWM_SetPwmLdok(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_0, true);
				LLD_PWM_StartTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_0);
			}
			else if(ts_lld_pwm_manager[e_Pwm].e_submodule == kPWM_Module_1)
			{
				LLD_PWM_SetPwmLdok(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_1, true);
				LLD_PWM_StartTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_1);
			}
			else if(ts_lld_pwm_manager[e_Pwm].e_submodule == kPWM_Module_2)
			{
				LLD_PWM_SetPwmLdok(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_2, true);
				LLD_PWM_StartTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_2);
			}
			else
			{
				LLD_PWM_SetPwmLdok(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_3, true);
				LLD_PWM_StartTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_3);
			}
		}
	}
	else
	{
		if((ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_A == 0) && (ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_B == 0))
		{
			if(ts_lld_pwm_manager[e_Pwm].e_submodule == kPWM_Module_0)
			{
				LLD_PWM_StopTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_0);
			}
			else if(ts_lld_pwm_manager[e_Pwm].e_submodule == kPWM_Module_1)
			{
				LLD_PWM_StopTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_1);
			}
			else if(ts_lld_pwm_manager[e_Pwm].e_submodule == kPWM_Module_2)
			{
				LLD_PWM_StopTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_2);
			}
			else
			{
				LLD_PWM_StopTimer(ts_lld_pwm_manager[e_Pwm].ps_Port, kPWM_Control_Module_3);
			}
		}

	}
}

/**
* @brief		Updates the PWM signal's duty cycle
* @param		e_Pwm : PWM number to update duty cycle
* @param		e_Channel : Channel A or B
* @param		u8_Dutycycle : Duty cycle to set between 0 and 100 (percent)
* @return		void
* @details
**/
void LLD_PWM_SetDutyCycle(typ_Lld_Pwm e_Pwm, typ_Lld_Pwm_Channel e_Channel, uint8_t u8_Dutycycle)
{
	if(e_Channel == LLD_PWM_CHANNEL_A)
	{
		ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_A = u8_Dutycycle;
	}
	else /* if(e_Channel == LLD_PWM_CHANNEL_B) */
	{
		ts_lld_pwm_manager[e_Pwm].u8_dutyCycle_B = u8_Dutycycle;
	}
}
