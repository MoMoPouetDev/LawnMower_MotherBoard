/*
 * RUN_Mower.h
 *
 *  Created on: 05 MAR 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_MOWER_RUN_MOWER_H_
#define RUN_RUN_MOWER_RUN_MOWER_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
typedef enum {
    UNKNOWN_ETAT = 0x00,
    TACHE_EN_COURS = 0x01,
    RETOUR_STATION = 0x02,
    EN_CHARGE = 0x03,
    PAS_DE_TACHE_EN_COURS = 0x04,
    PAUSE = 0x05
}EtatMower;

typedef enum {
    NTR = 0x10,
    BLOCKED_MOWER = 0x20,
    DETECTED_RAIN = 0x30,
    WIRE_NOT_DETECTED = 0x40,
    LOW_BATTERY = 0x50,
    VERY_LOW_BATTERY = 0x60,
    EMPTY_BATTERY = 0x70
}ErrorMower;

typedef struct {
    uint8_t degrees;
    uint8_t minutes;
    uint8_t decimalMSB;
    uint8_t decimalB;
    uint8_t decimalLSB;
}Coordinates;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_Mower_Init(void);
uint8_t RUN_Mower_LeaveDockCharger(void);
void RUN_Mower_GetAngles(void);
void RUN_Mower_GetAzimut(void); 
uint8_t RUN_Mower_RunMower(void);
void RUN_Mower_SonarDistance(void);
void RUN_Mower_TiltProtection(void);
uint8_t RUN_Mower_WireDetection(void);
uint8_t RUN_Mower_WireDetectionOnReturn(void);
uint8_t RUN_Mower_BumperDetection(void);
uint8_t RUN_Mower_DirectionFromBase(void);
uint8_t RUN_Mower_WireGuiding(void);
void RUN_Mower_SetEtatMower(EtatMower _eEtatMower);
void RUN_Mower_SetErrorMower(ErrorMower _eErrorMower);
EtatMower RUN_Mower_GetEtatMower(void);
ErrorMower RUN_Mower_GetErrorMower(void);
uint8_t RUN_Mower_IsTimeToMow(void);

#endif /* RUN_RUN_MOWER_RUN_MOWER_H_ */
