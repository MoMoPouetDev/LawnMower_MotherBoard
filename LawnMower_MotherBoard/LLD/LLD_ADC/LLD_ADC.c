/**
 * @file LLD_ADC.c
 * @author ACR
 * @brief Specific ADC driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_ADC_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include "MIMXRT1062.h"
#include "MIMXRT1062_features.h"

#include "fsl_clock.h"
#include "LLD_ADC.h"
//#include "LLD_TIMER.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES ADC_ETC ...                                                */
/*--------------------------------------------------------------------------*/

typedef enum
{
	ADC_TRIG0,
	ADC_TRIG1,
	ADC_TRIG2,
	ADC_TRIG3,
	ADC_TRIG4,
	ADC_TRIG5,
	ADC_TRIG6,
	ADC_TRIG7,
	NB_TRIG
}typ_Lld_Adc_Trigger;

/*! @brief The mask of status flags cleared by writing 1. */
#define ADC_ETC_DMA_CTRL_TRGn_REQ_MASK 0xFF0000U

/*!
 * @brief ADC_ETC customized status flags mask.
 */
enum _adc_etc_status_flag_mask
{
    kADC_ETC_Done0StatusFlagMask = 1U << 0U,
    kADC_ETC_Done1StatusFlagMask = 1U << 1U,
    kADC_ETC_Done2StatusFlagMask = 1U << 2U,
    kADC_ETC_ErrorStatusFlagMask  = 1U << 3U,
};

/*!
 * @brief External triggers sources.
 */
typedef enum _adc_etc_external_trigger_source
{
    /* External XBAR sources. Support HW or SW mode. */
    kADC_ETC_Trg0TriggerSource = 0U, /* External XBAR trigger0 source. */
    kADC_ETC_Trg1TriggerSource = 1U, /* External XBAR trigger1 source. */
    kADC_ETC_Trg2TriggerSource = 2U, /* External XBAR trigger2 source. */
    kADC_ETC_Trg3TriggerSource = 3U, /* External XBAR trigger3 source. */
    kADC_ETC_Trg4TriggerSource = 4U, /* External XBAR trigger4 source. */
    kADC_ETC_Trg5TriggerSource = 5U, /* External XBAR trigger5 source. */
    kADC_ETC_Trg6TriggerSource = 6U, /* External XBAR trigger6 source. */
    kADC_ETC_Trg7TriggerSource = 7U, /* External XBAR trigger7 source. */
    /* External TSC sources. Only support HW mode. */
    kADC_ETC_TSC0TriggerSource = 8U, /* External TSC trigger0 source. */
    kADC_ETC_TSC1TriggerSource = 9U, /* External TSC trigger1 source. */
} adc_etc_external_trigger_source_t;

/*!
 * @brief Interrupt enable/disable mask.
 */
typedef enum _adc_etc_interrupt_enable
{
    kADC_ETC_InterruptDisable     = 0U, /* Disable the ADC_ETC interrupt. */
    kADC_ETC_Done0InterruptEnable = 1U, /* Enable the DONE0 interrupt when ADC conversions complete. */
    kADC_ETC_Done1InterruptEnable = 2U, /* Enable the DONE1 interrupt when ADC conversions complete. */
    kADC_ETC_Done2InterruptEnable = 3U, /* Enable the DONE2 interrupt when ADC conversions complete. */
} adc_etc_interrupt_enable_t;

/*!
 * @brief ADC_ETC configuration.
 */
typedef struct _adc_etc_config
{
    uint32_t XBARtriggerMask;     /* Enable the corresponding trigger source. Available range is trigger0:0x01 to
                                     trigger7:0x80
                                     For example, XBARtriggerMask = 0x7U, which means trigger0, trigger1 and trigger2 is
                                     enabled. */
} adc_etc_config_t;

/*!
 * @brief ADC_ETC trigger chain configuration.
 */
typedef struct _adc_etc_trigger_chain_config
{
    bool enableB2BMode;           /* Enable ADC_ETC BackToBack mode. when not enabled B2B mode,
                                     wait until interval delay is reached. */
    uint32_t ADCHCRegisterSelect; /* Select relevant ADC_HCx register to trigger. 1U : HC0, 2U: HC1, 4U: HC2 ... */
    uint32_t ADCChannelSelect;    /* Select ADC sample channel. */
    adc_etc_interrupt_enable_t InterruptEnable; /* Enable/disable Interrupt. */
} adc_etc_trigger_chain_config_t;

/*!
 * @brief ADC_ETC trigger configuration.
 */
typedef struct _adc_etc_trigger_config
{
    uint32_t triggerChainLength;  /* TRIG chain length to the ADC. 0: Trig length is 1. ... 7: Trig length is 8. */
    uint32_t triggerPriority;     /* External trigger priority, 7 is highest, 0 is lowest. */
} adc_etc_trigger_config_t;

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ADC ...                                                    */
/*--------------------------------------------------------------------------*/

/*!
 * @brief Converter's status flags.
 */
typedef enum _adc_status_flags
{
    kADC_ConversionActiveFlag  = ADC_GS_ADACT_MASK, /*!< Conversion is active,not support w1c. */
    kADC_CalibrationFailedFlag = ADC_GS_CALF_MASK,  /*!< Calibration is failed,support w1c. */
    kADC_AsynchronousWakeupInterruptFlag = ADC_GS_AWKST_MASK, /*!< Asynchronous wakeup interrupt occurred, support w1c. */
} adc_status_flags_t;

/*!
 * @brief Sample time duration.
 */
typedef enum _adc_sample_period_mode
{
    /* This group of enumeration is for internal use which is related to register setting. */
    kADC_SamplePeriod2or12Clocks = 0U, /*!< Long sample 12 clocks or short sample 2 clocks. */
    kADC_SamplePeriod4or16Clocks = 1U, /*!< Long sample 16 clocks or short sample 4 clocks. */
    kADC_SamplePeriod6or20Clocks = 2U, /*!< Long sample 20 clocks or short sample 6 clocks. */
    kADC_SamplePeriod8or24Clocks = 3U, /*!< Long sample 24 clocks or short sample 8 clocks. */
    /* This group of enumeration is for a public user. */
    /* For long sample mode. */
    kADC_SamplePeriodLong12Clcoks = kADC_SamplePeriod2or12Clocks, /*!< Long sample 12 clocks. */
    kADC_SamplePeriodLong16Clcoks = kADC_SamplePeriod4or16Clocks, /*!< Long sample 16 clocks. */
    kADC_SamplePeriodLong20Clcoks = kADC_SamplePeriod6or20Clocks, /*!< Long sample 20 clocks. */
    kADC_SamplePeriodLong24Clcoks = kADC_SamplePeriod8or24Clocks, /*!< Long sample 24 clocks. */
    /* For short sample mode. */
    kADC_SamplePeriodShort2Clocks = kADC_SamplePeriod2or12Clocks, /*!< Short sample 2 clocks. */
    kADC_SamplePeriodShort4Clocks = kADC_SamplePeriod4or16Clocks, /*!< Short sample 4 clocks. */
    kADC_SamplePeriodShort6Clocks = kADC_SamplePeriod6or20Clocks, /*!< Short sample 6 clocks. */
    kADC_SamplePeriodShort8Clocks = kADC_SamplePeriod8or24Clocks, /*!< Short sample 8 clocks. */
} adc_sample_period_mode_t;

/*!
 * @brief Clock source.
 */
typedef enum _adc_clock_source
{
    kADC_ClockSourceIPG     = 0U, /*!< Select IPG clock to generate ADCK. */
    kADC_ClockSourceIPGDiv2 = 1U, /*!< Select IPG clock divided by 2 to generate ADCK. */
    kADC_ClockSourceAD = 3U, /*!< Select Asynchronous clock to generate ADCK. */
} adc_clock_source_t;

/*!
 * @brief Clock divider for the converter.
 */
typedef enum _adc_clock_drvier
{
    kADC_ClockDriver1 = 0U, /*!< For divider 1 from the input clock to the module. */
    kADC_ClockDriver2 = 1U, /*!< For divider 2 from the input clock to the module. */
    kADC_ClockDriver4 = 2U, /*!< For divider 4 from the input clock to the module. */
    kADC_ClockDriver8 = 3U, /*!< For divider 8 from the input clock to the module. */
} adc_clock_driver_t;

/*!
 * @brief Converter's resolution.
 */
typedef enum _adc_resolution
{
    kADC_Resolution8Bit  = 0U, /*!< Single End 8-bit resolution. */
    kADC_Resolution10Bit = 1U, /*!< Single End 10-bit resolution. */
    kADC_Resolution12Bit = 2U, /*!< Single End 12-bit resolution. */
} adc_resolution_t;

/*!
 * @brief Converter hardware average mode.
 */
typedef enum _adc_hardware_average_mode
{
    kADC_HardwareAverageCount4   = 0U, /*!< For hardware average with 4 samples. */
    kADC_HardwareAverageCount8   = 1U, /*!< For hardware average with 8 samples. */
    kADC_HardwareAverageCount16  = 2U, /*!< For hardware average with 16 samples. */
    kADC_HardwareAverageCount32  = 3U, /*!< For hardware average with 32 samples. */
    kADC_HardwareAverageDiasable = 4U, /*!< Disable the hardware average function. */
} adc_hardware_average_mode_t;

/*!
 * @brief Converter configuration.
 */
typedef struct _adc_config
{
    bool enableLongSample;                    		/*!< Enable the long sample mode. */
    adc_sample_period_mode_t samplePeriodMode; 		/*!< Select the sample period in long sample mode or short mode. */
    adc_resolution_t resolution;    				/*!< Select the ADC resolution mode. */
} adc_config_t;

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
/* ... DATATYPES LLD ADC ...                                                */
/*--------------------------------------------------------------------------*/

typedef struct
{
    lld_adc_transfer_callback_t callback; /*!< Callback function. */
}lld_adc_handle_t;

typedef struct
{
	IRQn_Type e_Irq;
	adc_etc_interrupt_enable_t e_InterruptFlag;
	lld_adc_handle_t s_ldd_adc_handle;
	const uint32_t *ps_AdcChannelConfig;
	uint8_t u8_AdcNbChannel;
}typ_Lld_Adc_Trigger_manager;

typ_Lld_Adc_Trigger_manager ts_Lld_Adc_Trigger_manager[NB_TRIG];

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

void ADC_Init(ADC_Type *base, const adc_config_t *config);
void LLD_ADC_SetChannelConfig(ADC_Type *base, uint32_t channelGroup);
status_t LLD_ADC_DoAutoCalibration(ADC_Type *base);
void LLD_ADC_SetHardwareAverageConfig(ADC_Type *base, adc_hardware_average_mode_t mode);
static inline uint32_t LLD_ADC_GetChannelConversionValue(ADC_Type *base, uint32_t channelGroup);
static inline uint32_t LLD_ADC_GetChannelStatusFlags(ADC_Type *base, uint32_t channelGroup);
static inline void LLD_ADC_EnableHardwareTrigger(ADC_Type *base, bool enable);
static inline uint32_t LLD_ADC_GetStatusFlags(ADC_Type *base);

void LLD_ADC_XBARA_Init(XBARA_Type *base);
void LLD_ADC_XBARA_SetSignalsConnection(XBARA_Type *base, xbar_input_signal_t input, xbar_output_signal_t output);

void LLD_ADC_ETC_Init(ADC_ETC_Type *base, const adc_etc_config_t *config);
void LLD_ADC_ETC_SetTriggerConfig(ADC_ETC_Type *base, uint32_t triggerGroup, const adc_etc_trigger_config_t *config);
void LLD_ADC_ETC_SetTriggerChainConfig(ADC_ETC_Type *base,
                                   uint32_t triggerGroup,
                                   uint32_t chainGroup,
                                   const adc_etc_trigger_chain_config_t *config);
uint32_t LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC_Type *base, adc_etc_external_trigger_source_t sourceIndex);
void LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC_Type *base, adc_etc_external_trigger_source_t sourceIndex, uint32_t mask);
uint32_t LLD_ADC_ETC_GetADCConversionValue(ADC_ETC_Type *base, uint32_t triggerGroup, uint32_t chainGroup);
static inline void LLD_ADC_ETC_DoSoftwareReset(ADC_ETC_Type *base, bool enable);

void LLD_ADC_InitChannel(typ_Lld_Adc_Trigger e_Trigger, const uint32_t* ps_AdcChannelConfig, bool enableInt);
void LLD_ADC_InitTriggerConfig(typ_Lld_Adc_Trigger e_Trigger);

void LLD_ADC_HandleIRQ(uint32_t mask);
void ADC_ETC_IRQ0_IRQHandler(void);
void ADC_ETC_IRQ1_IRQHandler(void);
void ADC_ETC_IRQ2_IRQHandler(void);

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*!
 * brief Initialize the ADC module.
 *
 * param base ADC peripheral base address.
 * param config Pointer to "adc_config_t" structure.
 */
void ADC_Init(ADC_Type *base, const adc_config_t *config)
{
    assert(NULL != config);

    uint32_t tmp32;

    /* Enable the clock. */
    if(base == ADC1)
    {
    	CLOCK_EnableClock(kCLOCK_Adc1);
    }
    else /* if(base == ADC2) */
    {
    	CLOCK_EnableClock(kCLOCK_Adc2);
    }

    /* ADCx_CFG */
    tmp32 = base->CFG & (ADC_CFG_AVGS_MASK | ADC_CFG_ADTRG_MASK); /* Reserve AVGS and ADTRG bits. */
    tmp32 |= ADC_CFG_ADSTS(config->samplePeriodMode) | (kADC_ClockSourceAD << ADC_CFG_ADICLK_SHIFT) |
    		(kADC_ClockDriver1 << ADC_CFG_ADIV_SHIFT) | ADC_CFG_MODE(config->resolution);
    if (config->enableLongSample)
   	{
   		tmp32 |= ADC_CFG_ADLSMP_MASK;
   	}
    base->CFG = tmp32;

    /* ADCx_GC  */
    tmp32 = base->GC & ~(ADC_GC_ADCO_MASK | ADC_GC_ADACKEN_MASK);
    /* Continuous conversion enabled */
    //tmp32 |= ADC_GC_ADCO_MASK;

    tmp32 |= ADC_GC_ADACKEN_MASK;
    base->GC = tmp32;
}

/*!
 * brief Configures the conversion channel.
 *
 * This operation triggers the conversion when in software trigger mode. When in hardware trigger mode, this API
 * configures the channel while the external trigger source helps to trigger the conversion.
 *
 * Note that the "Channel Group" has a detailed description.
 * To allow sequential conversions of the ADC to be triggered by internal peripherals, the ADC has more than one
 * group of status and control registers, one for each conversion. The channel group parameter indicates which group of
 * registers are used, for example channel group 0 is for Group A registers and channel group 1 is for Group B
 * registers. The
 * channel groups are used in a "ping-pong" approach to control the ADC operation.  At any point, only one of
 * the channel groups is actively controlling ADC conversions. The channel group 0 is used for both software and
 * hardware
 * trigger modes. Channel groups 1 and greater indicate potentially multiple channel group registers for
 * use only in hardware trigger mode. See the chip configuration information in the appropriate MCU reference manual
 * about the
 * number of SC1n registers (channel groups) specific to this device.  None of the channel groups 1 or greater are used
 * for software trigger operation. Therefore, writing to these channel groups does not initiate a new conversion.
 * Updating the channel group 0 while a different channel group is actively controlling a conversion is allowed and
 * vice versa. Writing any of the channel group registers while that specific channel group is actively controlling a
 * conversion aborts the current conversion.
 *
 * param base          ADC peripheral base address.
 * param channelGroup  Channel group index.
 */
void LLD_ADC_SetChannelConfig(ADC_Type *base, uint32_t channelGroup)
{
    assert(channelGroup < (uint32_t)FSL_FEATURE_ADC_CONVERSION_CONTROL_COUNT);

    uint32_t tmp32;

    tmp32 = ADC_HC_ADCH(16U);	/* 0b10000 : External channel selection from ADC_ETC */

    base->HC[channelGroup] = tmp32;
}

/*!
 * @brief  Gets the conversion value.
 *
 * @param  base         ADC peripheral base address.
 * @param  channelGroup Channel group index.
 *
 * @return              Conversion value.
 */
static inline uint32_t LLD_ADC_GetChannelConversionValue(ADC_Type *base, uint32_t channelGroup)
{
    assert(channelGroup < (uint32_t)FSL_FEATURE_ADC_CONVERSION_CONTROL_COUNT);

    return base->R[channelGroup];
}

/*
 *To complete calibration, the user must follow the below procedure:
 *  1. Configure ADC_CFG with actual operating values for maximum accuracy.
 *  2. Configure the ADC_GC values along with CAL bit.
 *  3. Check the status of CALF bit in ADC_GS and the CAL bit in ADC_GC.
 *  4. When CAL bit becomes '0' then check the CALF status and COCO[0] bit status.
 */
/*!
 * brief  Automates the hardware calibration.
 *
 * This auto calibration helps to adjust the plus/minus side gain automatically.
 * Execute the calibration before using the converter. Note that the software trigger should be used
 * during calibration.
 *
 * param  base ADC peripheral base address.
 *
 * return                 Execution status.
 * retval LLD_STATUS_Success Calibration is done successfully.
 * retval LLD_STATUS_Fail    Calibration has failed.
 */
status_t LLD_ADC_DoAutoCalibration(ADC_Type *base)
{
    status_t status = kStatus_Success;
    bool bHWTrigger = false;

    /* The calibration would be failed when in hardwar mode.
     * Remember the hardware trigger state here and restore it later if the hardware trigger is enabled.*/
    if (0U != (ADC_CFG_ADTRG_MASK & base->CFG))
    {
        bHWTrigger = true;
        LLD_ADC_EnableHardwareTrigger(base, false);
    }

    /* Clear the CALF and launch the calibration. */
    base->GS = ADC_GS_CALF_MASK; /* Clear the CALF. */
    base->GC |= ADC_GC_CAL_MASK; /* Launch the calibration. */

    /* Check the status of CALF bit in ADC_GS and the CAL bit in ADC_GC. */
    while (0U != (base->GC & ADC_GC_CAL_MASK))
    {
        /* Check the CALF when the calibration is active. */
        if (0U != (LLD_ADC_GetStatusFlags(base) & (uint32_t)kADC_CalibrationFailedFlag))
        {
            status = kStatus_Fail;
            break;
        }
    }

    /* When CAL bit becomes '0' then check the CALF status and COCO[0] bit status. */
    if (0U == LLD_ADC_GetChannelStatusFlags(base, 0U)) /* Check the COCO[0] bit status. */
    {
        status = kStatus_Fail;
    }
    if (0U != (LLD_ADC_GetStatusFlags(base) & (uint32_t)kADC_CalibrationFailedFlag)) /* Check the CALF status. */
    {
        status = kStatus_Fail;
    }

    /* Clear conversion done flag. */
    (void)LLD_ADC_GetChannelConversionValue(base, 0U);

    /* Restore original trigger mode. */
    if (true == bHWTrigger)
    {
        LLD_ADC_EnableHardwareTrigger(base, true);
    }

    return status;
}

/*!
 * brief Configures the hardware average mode.
 *
 * The hardware average mode provides a way to process the conversion result automatically by using hardware. The
 * multiple
 * conversion results are accumulated and averaged internally making them easier to read.
 *
 * param base ADC peripheral base address.
 * param mode Setting the hardware average mode. See "adc_hardware_average_mode_t".
 */
void LLD_ADC_SetHardwareAverageConfig(ADC_Type *base, adc_hardware_average_mode_t mode)
{
    uint32_t tmp32;

    if (mode == kADC_HardwareAverageDiasable)
    {
        base->GC &= ~ADC_GC_AVGE_MASK;
    }
    else
    {
        tmp32 = base->CFG & ~ADC_CFG_AVGS_MASK;
        tmp32 |= ADC_CFG_AVGS(mode);
        base->CFG = tmp32;
        base->GC |= ADC_GC_AVGE_MASK; /* Enable the hardware compare. */
    }
}

/*!
 * @brief Gets the status flags of channel.
 *
 * A conversion is completed when the result of the conversion is transferred into the data
 * result registers. (provided the compare function & hardware averaging is disabled), this is
 * indicated by the setting of COCOn. If hardware averaging is enabled, COCOn sets only,
 * if the last of the selected number of conversions is complete. If the compare function is
 * enabled, COCOn sets and conversion result data is transferred only if the compare
 * condition is true. If both hardware averaging and compare functions are enabled, then
 * COCOn sets only if the last of the selected number of conversions is complete and the
 * compare condition is true.
 *
 * @param base         ADC peripheral base address.
 * @param channelGroup Channel group index.
 *
 * @return             Status flags of channel.return 0 means COCO flag is 0,return 1 means COCOflag is 1.
 */
static inline uint32_t LLD_ADC_GetChannelStatusFlags(ADC_Type *base, uint32_t channelGroup)
{
    assert(channelGroup < (uint32_t)FSL_FEATURE_ADC_CONVERSION_CONTROL_COUNT);

    /* If flag is set,return 1,otherwise, return 0. */
    return (((base->HS) & (1UL << channelGroup)) >> channelGroup);
}

/*!
 * @brief Enables the hardware trigger mode.
 *
 * @param base ADC peripheral base address.
 * @param enable Switcher of the trigger mode. "true" means hardware tirgger mode,"false" means software mode.
 */
static inline void LLD_ADC_EnableHardwareTrigger(ADC_Type *base, bool enable)
{
    if (enable)
    {
        base->CFG |= ADC_CFG_ADTRG_MASK;
    }
    else
    {
        base->CFG &= ~ADC_CFG_ADTRG_MASK;
    }
}

/*!
 * @brief Gets the converter's status flags.
 *
 * @param base ADC peripheral base address.
 *
 * @return Flags' mask if indicated flags are asserted. See "adc_status_flags_t".
 */
static inline uint32_t LLD_ADC_GetStatusFlags(ADC_Type *base)
{
    return base->GS;
}

/*!
 * brief Initializes the XBARA module.
 *
 * This function un-gates the XBARA clock.
 *
 * param base XBARA peripheral address.
 */
void LLD_ADC_XBARA_Init(XBARA_Type *base)
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
 * Example:
   code
   LLD_ADC_XBARA_SetSignalsConnection(XBARA, kXBARA_InputPIT_TRG0, kXBARA_OutputDMAMUX18);
   endcode
 *
 * param base XBARA peripheral address.
 * param input XBARA input signal.
 * param output XBARA output signal.
 */
void LLD_ADC_XBARA_SetSignalsConnection(XBARA_Type *base, xbar_input_signal_t input, xbar_output_signal_t output)
{
    xbara_u8_u16_t regVal;
    uint8_t byteInReg;
    uint8_t outputIndex = (uint8_t)output;

    byteInReg = outputIndex % 2U;

    regVal._u16 = XBARA_SELx(base, outputIndex);

    regVal._u8[byteInReg] = (uint8_t)input;

    XBARA_SELx(base, outputIndex) = regVal._u16;
}

/*!
 * brief Initialize the ADC_ETC module.
 *
 * param base ADC_ETC peripheral base address.
 * param config Pointer to "adc_etc_config_t" structure.
 */
void LLD_ADC_ETC_Init(ADC_ETC_Type *base, const adc_etc_config_t *config)
{
    assert(NULL != config);

    base->CTRL = 0U;

    /* Disable software reset. */
    LLD_ADC_ETC_DoSoftwareReset(base, false);

    /* Set ADC_ETC_CTRL register. */
    base->CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(config->XBARtriggerMask);
}



/*!
 * brief Set the external XBAR trigger configuration.
 *
 * param base ADC_ETC peripheral base address.
 * param triggerGroup Trigger group index.
 * param config Pointer to "adc_etc_trigger_config_t" structure.
 */
void LLD_ADC_ETC_SetTriggerConfig(ADC_ETC_Type *base, uint32_t triggerGroup, const adc_etc_trigger_config_t *config)
{
    assert(triggerGroup < ADC_ETC_TRIGn_CTRL_COUNT);
    assert(ADC_ETC_TRIGn_COUNTER_COUNT > triggerGroup);

    uint32_t tmp32 = 0U;

    /* Set ADC_ETC_TRGn_CTRL register. */
    tmp32 = ADC_ETC_TRIGn_CTRL_TRIG_CHAIN(config->triggerChainLength) |
            ADC_ETC_TRIGn_CTRL_TRIG_PRIORITY(config->triggerPriority);

    if(triggerGroup >= ADC_TRIG4)
    {
    	tmp32 |= ADC_ETC_TRIGn_CTRL_SYNC_MODE_MASK;
    }

    base->TRIG[triggerGroup].TRIGn_COUNTER = 0;
    base->TRIG[triggerGroup].TRIGn_CTRL = tmp32;
}

/*!
 * brief Set the external XBAR trigger priority.
 *
 * param base ADC_ETC peripheral base address.
 * param triggerGroup Trigger group index.
 * param u8_Priority Trigger priority from 0 and 7. 7 is the highest priority, while 0 is lowest.
 */
void LLD_ADC_ETC_SetPriority(ADC_ETC_Type *base, uint32_t triggerGroup, typ_Lld_Adc_IRQ_Priority u8_Priority)
{
    assert(triggerGroup < ADC_ETC_TRIGn_CTRL_COUNT);
    assert(ADC_ETC_TRIGn_COUNTER_COUNT > triggerGroup);

    uint32_t tmp32 = 0U;

    /* Set ADC_ETC_TRGn_CTRL register. */
    tmp32 = base->TRIG[triggerGroup].TRIGn_CTRL;
    tmp32 &= ~ADC_ETC_TRIGn_CTRL_TRIG_PRIORITY_MASK;
    tmp32 |= ADC_ETC_TRIGn_CTRL_TRIG_PRIORITY(u8_Priority);
    base->TRIG[triggerGroup].TRIGn_CTRL = tmp32;
}

/*!
 * brief Set the external XBAR trigger chain configuration.
 * For example, if triggerGroup is set to 0U and chainGroup is set to 1U, which means Trigger0 source's chain1 would be
 * configurated.
 *
 * param base ADC_ETC peripheral base address.
 * param triggerGroup Trigger group index. Available number is 0~7.
 * param chainGroup Trigger chain group index. Available number is 0~7.
 * param config Pointer to "adc_etc_trigger_chain_config_t" structure.
 */
void LLD_ADC_ETC_SetTriggerChainConfig(ADC_ETC_Type *base,
                                   uint32_t triggerGroup,
                                   uint32_t chainGroup,
                                   const adc_etc_trigger_chain_config_t *config)
{
    assert(triggerGroup < ADC_ETC_TRIGn_CTRL_COUNT);

    uint32_t tmp32     = 0U;
    uint32_t tmpReg    = 0U;
    uint8_t mRemainder = (uint8_t)(chainGroup % 2U);

    /*  Set ADC_ETC_TRIGn_CHAINm register. */
    tmp32 = ADC_ETC_TRIGn_CHAIN_1_0_HWTS0(config->ADCHCRegisterSelect) |
            ADC_ETC_TRIGn_CHAIN_1_0_CSEL0(config->ADCChannelSelect) |
            ADC_ETC_TRIGn_CHAIN_1_0_IE0(config->InterruptEnable);

    tmp32 |= ADC_ETC_TRIGn_CHAIN_1_0_B2B0_MASK;

    switch (chainGroup / 2U)
    {
        case 0U: /* Configurate trigger chain0 and chain 1. */
            tmpReg = base->TRIG[triggerGroup].TRIGn_CHAIN_1_0;
            if (mRemainder == 0U) /* Chain 0. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_1_0_CSEL0_MASK | ADC_ETC_TRIGn_CHAIN_1_0_HWTS0_MASK |
                            ADC_ETC_TRIGn_CHAIN_1_0_B2B0_MASK | ADC_ETC_TRIGn_CHAIN_1_0_IE0_MASK);
                tmpReg |= tmp32;
            }
            else /* Chain 1. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_1_0_CSEL1_MASK | ADC_ETC_TRIGn_CHAIN_1_0_HWTS1_MASK |
                            ADC_ETC_TRIGn_CHAIN_1_0_B2B1_MASK | ADC_ETC_TRIGn_CHAIN_1_0_IE1_MASK);
                tmpReg |= (tmp32 << ADC_ETC_TRIGn_CHAIN_1_0_CSEL1_SHIFT);
            }
            base->TRIG[triggerGroup].TRIGn_CHAIN_1_0 = tmpReg;
            break;
        case 1U: /* Configurate trigger chain2 and chain 3. */
            tmpReg = base->TRIG[triggerGroup].TRIGn_CHAIN_3_2;
            if (mRemainder == 0U) /* Chain 2. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_3_2_CSEL2_MASK | ADC_ETC_TRIGn_CHAIN_3_2_HWTS2_MASK |
                            ADC_ETC_TRIGn_CHAIN_3_2_B2B2_MASK | ADC_ETC_TRIGn_CHAIN_3_2_IE2_MASK);
                tmpReg |= tmp32;
            }
            else /* Chain 3. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_3_2_CSEL3_MASK | ADC_ETC_TRIGn_CHAIN_3_2_HWTS3_MASK |
                            ADC_ETC_TRIGn_CHAIN_3_2_B2B3_MASK | ADC_ETC_TRIGn_CHAIN_3_2_IE3_MASK);
                tmpReg |= (tmp32 << ADC_ETC_TRIGn_CHAIN_3_2_CSEL3_SHIFT);
            }
            base->TRIG[triggerGroup].TRIGn_CHAIN_3_2 = tmpReg;
            break;
        case 2U: /* Configurate trigger chain4 and chain 5. */
            tmpReg = base->TRIG[triggerGroup].TRIGn_CHAIN_5_4;
            if (mRemainder == 0U) /* Chain 4. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_5_4_CSEL4_MASK | ADC_ETC_TRIGn_CHAIN_5_4_HWTS4_MASK |
                            ADC_ETC_TRIGn_CHAIN_5_4_B2B4_MASK | ADC_ETC_TRIGn_CHAIN_5_4_IE4_MASK);
                tmpReg |= tmp32;
            }
            else /* Chain 5. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_5_4_CSEL5_MASK | ADC_ETC_TRIGn_CHAIN_5_4_HWTS5_MASK |
                            ADC_ETC_TRIGn_CHAIN_5_4_B2B5_MASK | ADC_ETC_TRIGn_CHAIN_5_4_IE5_MASK);
                tmpReg |= (tmp32 << ADC_ETC_TRIGn_CHAIN_5_4_CSEL5_SHIFT);
            }
            base->TRIG[triggerGroup].TRIGn_CHAIN_5_4 = tmpReg;
            break;
        case 3U: /* Configurate trigger chain6 and chain 7. */
            tmpReg = base->TRIG[triggerGroup].TRIGn_CHAIN_7_6;
            if (mRemainder == 0U) /* Chain 6. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_7_6_CSEL6_MASK | ADC_ETC_TRIGn_CHAIN_7_6_HWTS6_MASK |
                            ADC_ETC_TRIGn_CHAIN_7_6_B2B6_MASK | ADC_ETC_TRIGn_CHAIN_7_6_IE6_MASK);
                tmpReg |= tmp32;
            }
            else /* Chain 7. */
            {
                tmpReg &= ~(ADC_ETC_TRIGn_CHAIN_7_6_CSEL7_MASK | ADC_ETC_TRIGn_CHAIN_7_6_HWTS7_MASK |
                            ADC_ETC_TRIGn_CHAIN_7_6_B2B7_MASK | ADC_ETC_TRIGn_CHAIN_7_6_IE7_MASK);
                tmpReg |= (tmp32 << ADC_ETC_TRIGn_CHAIN_7_6_CSEL7_SHIFT);
            }
            base->TRIG[triggerGroup].TRIGn_CHAIN_7_6 = tmpReg;
            break;
        default:
            assert(false);
            break;
    }
}

/*!
 * brief Gets the interrupt status flags of external XBAR and TSC triggers.
 *
 * param base ADC_ETC peripheral base address.
 * param sourceIndex trigger source index.
 *
 * return Status flags mask of trigger. Refer to "_adc_etc_status_flag_mask".
 */
uint32_t LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC_Type *base, adc_etc_external_trigger_source_t sourceIndex)
{
    uint32_t tmp32 = 0U;

    if (((base->DONE0_1_IRQ) & ((uint32_t)ADC_ETC_DONE0_1_IRQ_TRIG0_DONE0_MASK << (uint32_t)sourceIndex)) != 0U)
    {
        tmp32 |= (uint32_t)kADC_ETC_Done0StatusFlagMask; /* Customized DONE0 status flags mask, which is defined in
                                                  fsl_adc_etc.h file. */
    }
    if (((base->DONE0_1_IRQ) & ((uint32_t)ADC_ETC_DONE0_1_IRQ_TRIG0_DONE1_MASK << (uint32_t)sourceIndex)) != 0U)
    {
        tmp32 |= (uint32_t)kADC_ETC_Done1StatusFlagMask; /* Customized DONE1 status flags mask, which is defined in
                                                  fsl_adc_etc.h file. */
    }
    if (((base->DONE2_3_ERR_IRQ) & ((uint32_t)ADC_ETC_DONE2_3_ERR_IRQ_TRIG0_DONE2_MASK << (uint32_t)sourceIndex)) != 0U)
    {
        tmp32 |= (uint32_t)kADC_ETC_Done2StatusFlagMask; /* Customized DONE2 status flags mask, which is defined in
                                                  fsl_adc_etc.h file. */
    }
    if (((base->DONE2_3_ERR_IRQ) & ((uint32_t)ADC_ETC_DONE2_3_ERR_IRQ_TRIG0_ERR_MASK << (uint32_t)sourceIndex)) != 0U)
    {
        tmp32 |= (uint32_t)kADC_ETC_ErrorStatusFlagMask; /* Customized ERROR status flags mask, which is defined in
                                                  fsl_adc_etc.h file. */
    }
    return tmp32;
}

/*!
 * brief Clears the ADC_ETC's interrupt status falgs.
 *
 * param base ADC_ETC peripheral base address.
 * param sourceIndex trigger source index.
 * param mask Status flags mask of trigger. Refer to "_adc_etc_status_flag_mask".
 */
void LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC_Type *base, adc_etc_external_trigger_source_t sourceIndex, uint32_t mask)
{
    if (0U != (mask & (uint32_t)kADC_ETC_Done0StatusFlagMask)) /* Write 1 to clear DONE0 status flags. */
    {
        base->DONE0_1_IRQ = ((uint32_t)ADC_ETC_DONE0_1_IRQ_TRIG0_DONE0_MASK << (uint32_t)sourceIndex);
    }
    if (0U != (mask & (uint32_t)kADC_ETC_Done1StatusFlagMask)) /* Write 1 to clear DONE1 status flags. */
    {
        base->DONE0_1_IRQ = ((uint32_t)ADC_ETC_DONE0_1_IRQ_TRIG0_DONE1_MASK << (uint32_t)sourceIndex);
    }
    if (0U != (mask & (uint32_t)kADC_ETC_Done2StatusFlagMask)) /* Write 1 to clear DONE2 status flags. */
    {
        base->DONE2_3_ERR_IRQ = ((uint32_t)ADC_ETC_DONE2_3_ERR_IRQ_TRIG0_DONE2_MASK << (uint32_t)sourceIndex);
    }                                                      /* FSL_FEATURE_ADC_ETC_HAS_TRIGm_CHAIN_a_b_IEn_EN */
    if (0U != (mask & (uint32_t)kADC_ETC_ErrorStatusFlagMask)) /* Write 1 to clear ERROR status flags. */
    {
        base->DONE2_3_ERR_IRQ = ((uint32_t)ADC_ETC_DONE2_3_ERR_IRQ_TRIG0_ERR_MASK << (uint32_t)sourceIndex);
    }
}

/*!
 * brief Get ADC conversion result from external XBAR sources.
 * For example, if triggerGroup is set to 0U and chainGroup is set to 1U, which means the API would
 * return Trigger0 source's chain1 conversion result.
 *
 * param base ADC_ETC peripheral base address.
 * param triggerGroup Trigger group index. Available number is 0~7.
 * param chainGroup Trigger chain group index. Available number is 0~7.
 * return ADC conversion result value.
 */
uint32_t LLD_ADC_ETC_GetADCConversionValue(ADC_ETC_Type *base, uint32_t triggerGroup, uint32_t chainGroup)
{
    assert(triggerGroup < ADC_ETC_TRIGn_RESULT_1_0_COUNT);

    uint32_t mADCResult;
    uint8_t mRemainder = (uint8_t)(chainGroup % 2U);

    switch (chainGroup / 2U)
    {
        case 0U:
            if (0U == mRemainder)
            {
                mADCResult = ADC_ETC_TRIGn_RESULT_1_0_DATA0_MASK & (base->TRIG[triggerGroup].TRIGn_RESULT_1_0);
            }
            else
            {
                mADCResult = (base->TRIG[triggerGroup].TRIGn_RESULT_1_0) >> ADC_ETC_TRIGn_RESULT_1_0_DATA1_SHIFT;
            }
            break;
        case 1U:
            if (0U == mRemainder)
            {
                mADCResult = ADC_ETC_TRIGn_RESULT_3_2_DATA2_MASK & (base->TRIG[triggerGroup].TRIGn_RESULT_3_2);
            }
            else
            {
                mADCResult = (base->TRIG[triggerGroup].TRIGn_RESULT_3_2) >> ADC_ETC_TRIGn_RESULT_3_2_DATA3_SHIFT;
            }
            break;
        case 2U:
            if (0U == mRemainder)
            {
                mADCResult = ADC_ETC_TRIGn_RESULT_5_4_DATA4_MASK & (base->TRIG[triggerGroup].TRIGn_RESULT_5_4);
            }
            else
            {
                mADCResult = (base->TRIG[triggerGroup].TRIGn_RESULT_5_4) >> ADC_ETC_TRIGn_RESULT_5_4_DATA5_SHIFT;
            }
            break;
        case 3U:
            if (0U == mRemainder)
            {
                mADCResult = ADC_ETC_TRIGn_RESULT_7_6_DATA6_MASK & (base->TRIG[triggerGroup].TRIGn_RESULT_7_6);
            }
            else
            {
                mADCResult = (base->TRIG[triggerGroup].TRIGn_RESULT_7_6) >> ADC_ETC_TRIGn_RESULT_7_6_DATA7_SHIFT;
            }
            break;
        default:
            mADCResult = 0U;
            assert(false);
            break;
    }
    return mADCResult;
}

/*!
 * @brief When enable, all logical will be reset.
 *
 * @param base ADC_ETC peripheral base address.
 * @param enable Enable/Disable the software reset.
 */
static inline void LLD_ADC_ETC_DoSoftwareReset(ADC_ETC_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL |= ADC_ETC_CTRL_SOFTRST_MASK;
    }
    else
    {
        base->CTRL &= ~ADC_ETC_CTRL_SOFTRST_MASK;
    }
}

/**
* @brief		Initialization of trigger configuration
* @param		e_Trigger : Trigger to configure from 0 to 7
* @return		void
* @details		By default : Priority 0
**/
void LLD_ADC_InitTriggerConfig(typ_Lld_Adc_Trigger e_Trigger)
{
	adc_etc_trigger_config_t adcEtcTriggerConfig;

	if(ts_Lld_Adc_Trigger_manager[e_Trigger].u8_AdcNbChannel > 0)
	{
		adcEtcTriggerConfig.triggerChainLength = ts_Lld_Adc_Trigger_manager[e_Trigger].u8_AdcNbChannel - 1;
		adcEtcTriggerConfig.triggerPriority = ADC_PRIORITY0;
		LLD_ADC_ETC_SetTriggerConfig(ADC_ETC, e_Trigger, &adcEtcTriggerConfig);
	}
	else
	{
		adcEtcTriggerConfig.triggerChainLength = 0;
		adcEtcTriggerConfig.triggerPriority = ADC_PRIORITY0;
		LLD_ADC_ETC_SetTriggerConfig(ADC_ETC, e_Trigger, &adcEtcTriggerConfig);
	}
}

/**
* @brief		ADC channel initialization
* @param		e_Trigger : Trigger to initialize from 0 to 7
* @param		ps_AdcChannelConfig : Pointer on channel configuration array (length must be 8)
* @param		enableInt : true : Enable end of conversion interruption, false : No interruption
* @return		void
* @details
**/
void LLD_ADC_InitChannel(typ_Lld_Adc_Trigger e_Trigger, const uint32_t* ps_AdcChannelConfig, bool enableInt)
{
	adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;
	uint8_t u8_Channel;

	if(ts_Lld_Adc_Trigger_manager[e_Trigger].u8_AdcNbChannel > 0)
	{
		/* Channel initialization */
		adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << e_Trigger;
		adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable;

		for(u8_Channel = 0; u8_Channel < ts_Lld_Adc_Trigger_manager[e_Trigger].u8_AdcNbChannel; u8_Channel++)
		{
			/* Configure channel n */
			adcEtcTriggerChainConfig.ADCChannelSelect = ts_Lld_Adc_Trigger_manager[e_Trigger].ps_AdcChannelConfig[u8_Channel];
			if(enableInt == true && (u8_Channel == (ts_Lld_Adc_Trigger_manager[e_Trigger].u8_AdcNbChannel - 1)))
			{
				adcEtcTriggerChainConfig.InterruptEnable = ts_Lld_Adc_Trigger_manager[e_Trigger].e_InterruptFlag;
			}
			LLD_ADC_ETC_SetTriggerChainConfig(ADC_ETC, e_Trigger, u8_Channel, &adcEtcTriggerChainConfig);
		}
	}
	else
	{
		/* Channel initialization */
		adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << e_Trigger;
		adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable;
		adcEtcTriggerChainConfig.ADCChannelSelect = 0;
		LLD_ADC_ETC_SetTriggerChainConfig(ADC_ETC, e_Trigger, 0, &adcEtcTriggerChainConfig);
	}
}

/**
* @brief		LLD ADC IRQ handle function.
* @param		u32_mask : Mask of IRQ
* @return		void
* @details		Clear the corresponding trigger flag
* 				Call the callback function
**/
void LLD_ADC_HandleIRQ(uint32_t u32_mask)
{
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg0TriggerSource) & u32_mask) == u32_mask)
	{
		/* Calling Callback Function if has one. */
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg0TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG0].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG0].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg1TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg1TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG1].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG1].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg2TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg2TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG2].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG2].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg3TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg3TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG3].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG3].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg4TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg4TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG0].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG0].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg5TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg5TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG1].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG1].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg6TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg6TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG2].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG2].s_ldd_adc_handle.callback();
		}
	}
	if((LLD_ADC_ETC_GetInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg7TriggerSource) & u32_mask) == u32_mask)
	{
		LLD_ADC_ETC_ClearInterruptStatusFlags(ADC_ETC, kADC_ETC_Trg7TriggerSource, u32_mask);
		if (ts_Lld_Adc_Trigger_manager[ADC_TRIG3].s_ldd_adc_handle.callback != NULL)
		{
			ts_Lld_Adc_Trigger_manager[ADC_TRIG3].s_ldd_adc_handle.callback();
		}
	}
}

/**
* @brief		ETC DONE0 Interruption
* @param		void
* @return		void
**/
void ADC_ETC_IRQ0_IRQHandler(void)
{
	LLD_ADC_HandleIRQ(kADC_ETC_Done0StatusFlagMask);
	SDK_ISR_EXIT_BARRIER;
}


/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		ADC initialization
* @param		e_EtcNumber : ETC number to initialize from 0 to 3 corresponding to PIT Trigger Input 0 to 3
* @param		ps_ChannelConfigAdc1 : Pointer on ADC1 channel configuration array
* @param		u8_Adc1_NbChannel : Number of ADC1 channel
* @param		ps_ChannelConfigAdc2 : Pointer on ADC2 channel configuration array
* @param		u8_Adc2_NbChannel : Number of ADC2 channel
* @return		void
* @details		PIT Triggers IN 0 to 3 are synchronized respectively with triggers chain 0 to 3 and chain 4 to 7
* 				By default : No average - 12 bits resolution - Short sample 2 clock
* 				All end conversion interruption is managed by ADC_ETC_IRQ0_IRQn (Done0)
* 				Priority Irq : ADC_PRIORITY0
**/
void LLD_ADC_Init(	typ_Lld_Adc_Etc e_EtcNumber,
					const uint32_t* ps_ChannelConfigAdc1, uint8_t u8_Adc1_NbChannel,
					const uint32_t* ps_ChannelConfigAdc2, uint8_t u8_Adc2_NbChannel)
{
	adc_config_t k_adcConfig;
    adc_etc_config_t adcEtcConfig;
    bool b_Adc1Int = false;
    bool b_Adc2Int = false;

	/* Init ADC modules */
	k_adcConfig.enableLongSample = false;
	k_adcConfig.samplePeriodMode = kADC_SamplePeriod2or12Clocks;
	k_adcConfig.resolution = kADC_Resolution12Bit;

	/* Initialize the ADC1 module. */
	ADC_Init(ADC1, &k_adcConfig);
	LLD_ADC_SetHardwareAverageConfig(ADC1, ADC_AVERAGE_DISABLE);
	LLD_ADC_EnableHardwareTrigger(ADC1, true);

	/* Initialize the ADC2 module. */
	ADC_Init(ADC2, &k_adcConfig);
	LLD_ADC_SetHardwareAverageConfig(ADC2, ADC_AVERAGE_DISABLE);
	LLD_ADC_EnableHardwareTrigger(ADC2, true);

	if(e_EtcNumber == ADC_ETC_PIT0)
	{
		LLD_ADC_SetChannelConfig(ADC1, ADC_TRIG0);
		LLD_ADC_DoAutoCalibration(ADC1);

		LLD_ADC_SetChannelConfig(ADC2, ADC_TRIG4);
		LLD_ADC_DoAutoCalibration(ADC2);
	}
	else if(e_EtcNumber == ADC_ETC_PIT1)
	{
		LLD_ADC_SetChannelConfig(ADC1, ADC_TRIG1);
		LLD_ADC_DoAutoCalibration(ADC1);

		LLD_ADC_SetChannelConfig(ADC2, ADC_TRIG5);
		LLD_ADC_DoAutoCalibration(ADC2);
	}
	else if(e_EtcNumber == ADC_ETC_PIT2)
	{
		LLD_ADC_SetChannelConfig(ADC1, ADC_TRIG2);
		LLD_ADC_DoAutoCalibration(ADC1);

		LLD_ADC_SetChannelConfig(ADC2, ADC_TRIG6);
		LLD_ADC_DoAutoCalibration(ADC2);
	}
	else /* if(e_EtcNumber == ADC_ETC_PIT3) */
	{
		LLD_ADC_SetChannelConfig(ADC1, ADC_TRIG3);
		LLD_ADC_DoAutoCalibration(ADC1);

		LLD_ADC_SetChannelConfig(ADC2, ADC_TRIG7);
		LLD_ADC_DoAutoCalibration(ADC2);
	}

	/* Init xbara module. */
	LLD_ADC_XBARA_Init(XBARA1);

	if(e_EtcNumber == ADC_ETC_PIT0)
	{
		/* Configure the XBARA signal connections between PIT0 and ADC_ETC */
		LLD_ADC_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputPitTrigger0, kXBARA1_OutputAdcEtcXbar0Trig0);
	}
	else if(e_EtcNumber == ADC_ETC_PIT1)
	{
		LLD_ADC_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputPitTrigger1, kXBARA1_OutputAdcEtcXbar0Trig1);
	}
	else if(e_EtcNumber == ADC_ETC_PIT2)
	{
		LLD_ADC_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputPitTrigger2, kXBARA1_OutputAdcEtcXbar0Trig2);
	}
	else /* if(e_EtcNumber == ADC_ETC_PIT3) */
	{
		LLD_ADC_XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputPitTrigger3, kXBARA1_OutputAdcEtcXbar0Trig3);
	}

	/* Initialize the ADC_ETC. ---------------------------------------------------------------------- */
	adcEtcConfig.XBARtriggerMask = 0x00;	/* All triggers disable */
	LLD_ADC_ETC_Init(ADC_ETC, &adcEtcConfig);

	if(u8_Adc2_NbChannel == 0)
	{
		b_Adc1Int = true;
	}
	else
	{
		b_Adc2Int = true;
	}

	if(e_EtcNumber == ADC_ETC_PIT0)
	{
		/* Trigger0 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG0].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG0].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG0].ps_AdcChannelConfig = ps_ChannelConfigAdc1;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG0].u8_AdcNbChannel = u8_Adc1_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG0);
		LLD_ADC_InitChannel(ADC_TRIG0, ps_ChannelConfigAdc1, b_Adc1Int);

		/* Trigger4 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG4].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG4].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG4].ps_AdcChannelConfig = ps_ChannelConfigAdc2;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG4].u8_AdcNbChannel = u8_Adc2_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG4);
		LLD_ADC_InitChannel(ADC_TRIG4, ps_ChannelConfigAdc2, b_Adc2Int);
	}
	else if(e_EtcNumber == ADC_ETC_PIT1)
	{
		/* Trigger1 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG1].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG1].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG1].ps_AdcChannelConfig = ps_ChannelConfigAdc1;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG1].u8_AdcNbChannel = u8_Adc1_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG1);
		LLD_ADC_InitChannel(ADC_TRIG1, ps_ChannelConfigAdc1, b_Adc1Int);
		/* Trigger5 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG5].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG5].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG5].ps_AdcChannelConfig = ps_ChannelConfigAdc2;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG5].u8_AdcNbChannel = u8_Adc2_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG5);
		LLD_ADC_InitChannel(ADC_TRIG5, ps_ChannelConfigAdc2, b_Adc2Int);
	}
	else if(e_EtcNumber == ADC_ETC_PIT2)
	{
		/* Trigger2 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG2].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG2].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG2].ps_AdcChannelConfig = ps_ChannelConfigAdc1;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG2].u8_AdcNbChannel = u8_Adc1_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG2);
		LLD_ADC_InitChannel(ADC_TRIG2, ps_ChannelConfigAdc1, b_Adc1Int);
		/* Trigger6 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG6].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG6].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG6].ps_AdcChannelConfig = ps_ChannelConfigAdc2;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG6].u8_AdcNbChannel = u8_Adc2_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG6);
		LLD_ADC_InitChannel(ADC_TRIG6, ps_ChannelConfigAdc2, b_Adc2Int);
	}
	else if(e_EtcNumber == ADC_ETC_PIT3)
	{
		/* Trigger3 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG3].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG3].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG3].ps_AdcChannelConfig = ps_ChannelConfigAdc1;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG3].u8_AdcNbChannel = u8_Adc1_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG3);
		LLD_ADC_InitChannel(ADC_TRIG3, ps_ChannelConfigAdc1, b_Adc1Int);
		/* Trigger7 configuration. */
		ts_Lld_Adc_Trigger_manager[ADC_TRIG7].e_Irq = ADC_ETC_IRQ0_IRQn;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG7].e_InterruptFlag = kADC_ETC_Done0InterruptEnable;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG7].ps_AdcChannelConfig = ps_ChannelConfigAdc2;
		ts_Lld_Adc_Trigger_manager[ADC_TRIG7].u8_AdcNbChannel = u8_Adc2_NbChannel;
		LLD_ADC_InitTriggerConfig(ADC_TRIG7);
		LLD_ADC_InitChannel(ADC_TRIG7, ps_ChannelConfigAdc2, b_Adc2Int);
	}
}

/**
* @brief		Enable/Disable ADC interrupt
* @param		e_Enable : see "typ_Lld_Adc_Enable" enum.
* @return		void
* @details
**/
void LLD_ADC_EnableIrq(typ_Lld_Adc_Enable e_Enable)
{
	if(e_Enable == ADC_ENABLE)
	{
		EnableIRQ(ADC_ETC_IRQ0_IRQn);
	}
	else
	{
		DisableIRQ(ADC_ETC_IRQ0_IRQn);
	}
}

/**
* @brief		Read ADC conversion value
* @param		e_EtcNumber : ETC number to configure, from 0 to 3, corresponding of PIT Trigger
* @param		e_AdcNumber : ADC module to read : 1 or 2
* @param		e_Channel : Number of Trigger channel : 0 to 7.
* @return		Conversion value
* @details
**/
uint32_t LLD_ADC_ReadConversionValue(typ_Lld_Adc_Etc e_EtcNumber, typ_Lld_Adc_AdcNumber e_AdcNumber, typ_Lld_Adc_Etc_Channel e_Channel)
{
	uint32_t u32_Value = 0;

	if(e_EtcNumber == ADC_ETC_PIT0)
	{
		if(e_AdcNumber == ADC_ETC_ADC1)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG0, e_Channel);
		}
		else if(e_AdcNumber == ADC_ETC_ADC2)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG4, e_Channel);
		}
	}
	else if(e_EtcNumber == ADC_ETC_PIT1)
	{
		if(e_AdcNumber == ADC_ETC_ADC1)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG1, e_Channel);
		}
		else if(e_AdcNumber == ADC_ETC_ADC2)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG5, e_Channel);
		}
	}
	else if(e_EtcNumber == ADC_ETC_PIT2)
	{
		if(e_AdcNumber == ADC_ETC_ADC1)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG2, e_Channel);
		}
		else if(e_AdcNumber == ADC_ETC_ADC2)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG6, e_Channel);
		}
	}
	else if(e_EtcNumber == ADC_ETC_PIT3)
	{
		if(e_AdcNumber == ADC_ETC_ADC1)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG3, e_Channel);
		}
		else if(e_AdcNumber == ADC_ETC_ADC2)
		{
			u32_Value = LLD_ADC_ETC_GetADCConversionValue(ADC_ETC, ADC_TRIG7, e_Channel);
		}
	}

	return u32_Value;
}

/**
* @brief		Enable/Disable conversion
* @param		e_Enable : see "typ_Lld_Adc_Enable" enum.
* @return		void
* @details		Enable/Disable all conversion
**/
void LLD_ADC_EnableConversion(typ_Lld_Adc_Enable e_Enable)
{
	adc_etc_config_t adcEtcConfig;

	if(e_Enable == ADC_ENABLE)
	{
		adcEtcConfig.XBARtriggerMask = 0x0F;
	}
	else
	{
		adcEtcConfig.XBARtriggerMask = 0x00;
	}

	ADC_ETC->CTRL = 0U;

	/* Set ADC_ETC_CTRL register. */
	ADC_ETC->CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(adcEtcConfig.XBARtriggerMask);
}

/**
* @brief		Set trigger priority
* @param		e_EtcNumber : ETC number to configure, from 0 to 3, corresponding of PIT Trigger
* @param		u8_Priority : trigger priority from 0 and 7. 7 is the highest priority, while 0 is lowest.
* @return		void
* @details
**/
void LLD_ADC_PriorityIrq(typ_Lld_Adc_Etc e_EtcNumber, typ_Lld_Adc_IRQ_Priority u8_Priority)
{
	typ_Lld_Adc_Trigger e_TriggerAdc1;
	typ_Lld_Adc_Trigger e_TriggerAdc2;

	if(e_EtcNumber == ADC_ETC_PIT0)
	{
		e_TriggerAdc1 = ADC_TRIG0;
		e_TriggerAdc2 = ADC_TRIG4;
	}
	else if(e_EtcNumber == ADC_ETC_PIT1)
	{
		e_TriggerAdc1 = ADC_TRIG1;
		e_TriggerAdc2 = ADC_TRIG5;
	}
	else if(e_EtcNumber == ADC_ETC_PIT2)
	{
		e_TriggerAdc1 = ADC_TRIG2;
		e_TriggerAdc2 = ADC_TRIG6;
	}
	else //if(e_EtcNumber == ADC_ETC_PIT3)
	{
		e_TriggerAdc1 = ADC_TRIG3;
		e_TriggerAdc2 = ADC_TRIG7;
	}

	LLD_ADC_ETC_SetPriority(ADC_ETC, e_TriggerAdc1, u8_Priority);
	LLD_ADC_ETC_SetPriority(ADC_ETC, e_TriggerAdc2, u8_Priority);
}

/**
* @brief		Set conversion rate
* @param		e_LongSampleTime : Sample time in number of cycles (3/5/7/9/13/17/21/25)
* @param		e_Resolution : Convert's resolution.
* @param		e_AverageMode : Number of samples used for a conversion.
* @return		void
* @details		For all ADC
**/
void LLD_ADC_SetConversionRate(typ_Lld_Adc_SampleTime e_LongSampleTime, typ_Lld_Adc_Resolution e_Resolution,  typ_Lld_Adc_Average e_AverageMode)
{
	adc_config_t k_adcConfig;
	uint32_t tmp32;

	LLD_ADC_SetHardwareAverageConfig(ADC1, e_AverageMode);
	LLD_ADC_SetHardwareAverageConfig(ADC2, e_AverageMode);
	k_adcConfig.resolution = e_Resolution;

	if(e_LongSampleTime == ADC_SAMPLE_TIME_3)
	{
		k_adcConfig.enableLongSample = false;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodShort2Clocks;
	}
	else if(e_LongSampleTime == ADC_SAMPLE_TIME_5)
	{
		k_adcConfig.enableLongSample = false;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodShort4Clocks;
	}
	else if(e_LongSampleTime == ADC_SAMPLE_TIME_7)
	{
		k_adcConfig.enableLongSample = false;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodShort6Clocks;
	}
	else if(e_LongSampleTime == ADC_SAMPLE_TIME_9)
	{
		k_adcConfig.enableLongSample = false;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodShort8Clocks;
	}
	if(e_LongSampleTime == ADC_SAMPLE_TIME_13)
	{
		k_adcConfig.enableLongSample = true;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodLong12Clcoks;
	}
	else if(e_LongSampleTime == ADC_SAMPLE_TIME_17)
	{
		k_adcConfig.enableLongSample = true;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodLong16Clcoks;
	}
	else if(e_LongSampleTime == ADC_SAMPLE_TIME_21)
	{
		k_adcConfig.enableLongSample = true;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodLong20Clcoks;
	}
	else /* if(e_LongSampleTime == ADC_SAMPLE_TIME_25) */
	{
		k_adcConfig.enableLongSample = true;
		k_adcConfig.samplePeriodMode = kADC_SamplePeriodLong24Clcoks;
	}

	/* ADC1 Configuration */
	/* ADCx_CFG */
	tmp32 = ADC1->CFG & (ADC_CFG_AVGS_MASK | ADC_CFG_ADTRG_MASK); /* Reserve AVGS and ADTRG bits. */
	tmp32 |= ADC_CFG_ADSTS(k_adcConfig.samplePeriodMode) | (kADC_ClockSourceAD << ADC_CFG_ADICLK_SHIFT) |
			(kADC_ClockDriver1 << ADC_CFG_ADIV_SHIFT) | ADC_CFG_MODE(k_adcConfig.resolution);
	if (k_adcConfig.enableLongSample)
	{
		tmp32 |= ADC_CFG_ADLSMP_MASK;
	}
	ADC1->CFG = tmp32;
	/* ADCx_GC  */
	tmp32 = ADC1->GC & ~(ADC_GC_ADCO_MASK | ADC_GC_ADACKEN_MASK);
	tmp32 |= ADC_GC_ADACKEN_MASK;
	ADC1->GC = tmp32;

	/* ADC2 Configuration */
	/* ADCx_CFG */
	tmp32 = ADC2->CFG & (ADC_CFG_AVGS_MASK | ADC_CFG_ADTRG_MASK); /* Reserve AVGS and ADTRG bits. */
	tmp32 |= ADC_CFG_ADSTS(k_adcConfig.samplePeriodMode) | (kADC_ClockSourceAD << ADC_CFG_ADICLK_SHIFT) |
			(kADC_ClockDriver1 << ADC_CFG_ADIV_SHIFT) | ADC_CFG_MODE(k_adcConfig.resolution);
	if (k_adcConfig.enableLongSample)
	{
		tmp32 |= ADC_CFG_ADLSMP_MASK;
	}
	ADC2->CFG = tmp32;
	/* ADCx_GC  */
	tmp32 = ADC2->GC & ~(ADC_GC_ADCO_MASK | ADC_GC_ADACKEN_MASK);
	tmp32 |= ADC_GC_ADACKEN_MASK;
	ADC2->GC = tmp32;
}

/**
* @brief		Set callback function
* @param		e_EtcNumber : ETC number to configure, from 0 to 3, corresponding of PIT Trigger
* @param		pf_callback : Pointer on function
* @return		void
* @details
**/
void LLD_ADC_SetCallback(typ_Lld_Adc_Etc e_EtcNumber, lld_adc_transfer_callback_t pf_callback)
{
	typ_Lld_Adc_Trigger e_TriggerAdc1;
	typ_Lld_Adc_Trigger e_TriggerAdc2;

	if(e_EtcNumber == ADC_ETC_PIT0)
	{
		e_TriggerAdc1 = ADC_TRIG0;
		e_TriggerAdc2 = ADC_TRIG4;
	}
	else if(e_EtcNumber == ADC_ETC_PIT1)
	{
		e_TriggerAdc1 = ADC_TRIG1;
		e_TriggerAdc2 = ADC_TRIG5;
	}
	else if(e_EtcNumber == ADC_ETC_PIT2)
	{
		e_TriggerAdc1 = ADC_TRIG2;
		e_TriggerAdc2 = ADC_TRIG6;
	}
	else //if(e_EtcNumber == ADC_ETC_PIT3)
	{
		e_TriggerAdc1 = ADC_TRIG3;
		e_TriggerAdc2 = ADC_TRIG7;
	}

	ts_Lld_Adc_Trigger_manager[e_TriggerAdc1].s_ldd_adc_handle.callback = pf_callback;
	ts_Lld_Adc_Trigger_manager[e_TriggerAdc2].s_ldd_adc_handle.callback = pf_callback;
}
