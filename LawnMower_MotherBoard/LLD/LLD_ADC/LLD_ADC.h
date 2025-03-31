/**
 * @file LLD_ADC.h
 * @author ACR
 * @brief Header file for ADC peripheral
 * @details
**/

#ifndef LLD_ADC_H_
#define LLD_ADC_H_

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

typedef enum
{
	ADC_ETC_ADC1,
	ADC_ETC_ADC2,
}typ_Lld_Adc_AdcNumber;

typedef enum
{
	ADC_ETC_PIT0,
	ADC_ETC_PIT1,
	ADC_ETC_PIT2,
	ADC_ETC_PIT3,
}typ_Lld_Adc_Etc;

typedef enum
{
	ADC_CHANNEL0,
	ADC_CHANNEL1,
	ADC_CHANNEL2,
	ADC_CHANNEL3,
	ADC_CHANNEL4,
	ADC_CHANNEL5,
	ADC_CHANNEL6,
	ADC_CHANNEL7,
}typ_Lld_Adc_Etc_Channel;

typedef enum
{
	ADC_PRIORITY0,
	ADC_PRIORITY1,
	ADC_PRIORITY2,
	ADC_PRIORITY3,
	ADC_PRIORITY4,
	ADC_PRIORITY5,
	ADC_PRIORITY6,
	ADC_PRIORITY7,
}typ_Lld_Adc_IRQ_Priority;

typedef enum
{
	ADC_AVERAGE_COUNT4,
	ADC_AVERAGE_COUNT8,
	ADC_AVERAGE_COUNT16,
	ADC_AVERAGE_COUNT32,
	ADC_AVERAGE_DISABLE,
}typ_Lld_Adc_Average;

typedef enum
{
	ADC_RESOLUTION_8BITS,
	ADC_RESOLUTION_10BITS,
	ADC_RESOLUTION_12BITS,
}typ_Lld_Adc_Resolution;

typedef enum
{
	ADC_SAMPLE_TIME_3,
	ADC_SAMPLE_TIME_5,
	ADC_SAMPLE_TIME_7,
	ADC_SAMPLE_TIME_9,
	ADC_SAMPLE_TIME_13,
	ADC_SAMPLE_TIME_17,
	ADC_SAMPLE_TIME_21,
	ADC_SAMPLE_TIME_25,
}typ_Lld_Adc_SampleTime;

typedef enum
{
	ADC_DISABLE,
	ADC_ENABLE,
}typ_Lld_Adc_Enable;

typedef void (*lld_adc_transfer_callback_t)(void);

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/

void LLD_ADC_Init(	typ_Lld_Adc_Etc e_EtcNumber,
					const uint32_t* ps_ChannelConfigAdc1, uint8_t u8_Adc1_NbChannel,
					const uint32_t* ps_ChannelConfigAdc2, uint8_t u8_Adc2_NbChannel);

void LLD_ADC_SetConversionRate(typ_Lld_Adc_SampleTime e_LongSampleTime, typ_Lld_Adc_Resolution e_Resolution, typ_Lld_Adc_Average e_AverageMode);

void LLD_ADC_SetCallback(typ_Lld_Adc_Etc e_EtcNumber, lld_adc_transfer_callback_t pf_callback);
void LLD_ADC_PriorityIrq(typ_Lld_Adc_Etc e_EtcNumber, typ_Lld_Adc_IRQ_Priority u8_Priority);
void LLD_ADC_EnableIrq(typ_Lld_Adc_Enable e_Enable);

void LLD_ADC_EnableConversion(typ_Lld_Adc_Enable e_Enable);

uint32_t LLD_ADC_ReadConversionValue(typ_Lld_Adc_Etc e_EtcNumber, typ_Lld_Adc_AdcNumber e_AdcNumber, typ_Lld_Adc_Etc_Channel e_Channel);

#endif /* LLD_ADC_H_ */
