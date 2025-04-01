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

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/
void LLD_ADC_Init(void);
uint8_t LLD_ADC_ReadConversionValue(uint8_t u8_adcChannel, uint16_t* pu16_adcValue);

#endif /* LLD_ADC_H_ */
