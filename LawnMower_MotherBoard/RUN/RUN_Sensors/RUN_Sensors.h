/*
 * RUN_Sensors.h
 *
 *  Created on: 25 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_SENSORS_RUN_SENSORS_H_
#define RUN_RUN_SENSORS_RUN_SENSORS_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "HAL_GPIO.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
/*** Capteur Tension ***/
#define CHARGING_THRESHOLD 3225
#define SENSOR_V_OK 80
#define SENSOR_V_FAIBLE_WARN 20
#define SENSOR_V_FAIBLE_ERR 10
#define SENSOR_V_EMPTY 1

#define ERROR_DATA 0xFF

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_Sensors_Init(void);
uint8_t RUN_Sensors_IsTimeToMow(void);
uint8_t RUN_Sensors_IsCharging(void);
int8_t RUN_Sensors_IsEnoughCharged(void);
uint8_t RUN_Sensors_GetBatteryPercent(void);
Etat RUN_Sensors_GetRainState(void);
Etat RUN_Sensors_GetDockState(void);
void RUN_Sensors_SetRainState(Etat e_rainState);

#endif /* RUN_RUN_SENSORS_RUN_SENSORS_H_ */
