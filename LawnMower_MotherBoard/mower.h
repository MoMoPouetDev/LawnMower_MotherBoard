//
//  mower.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 06/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef mower_h
#define mower_h

typedef struct {
    uint8_t degrees;
    uint8_t minutes;
    uint8_t decimalMSB;
    uint8_t decimalB;
    uint8_t decimalLSB;
}Coordinates;

uint8_t isDocking(void);
uint8_t isCharging(void);
uint8_t isTimeToMow(void);
uint8_t isEnoughCharged(void);
uint8_t isRaining(void);
void MOWER_startMower(void);
void MOWER_goDockCharger(void);
void MOWER_directionFromBase(void);
void MOWER_pidController(uint8_t*);
void MOWER_leaveDockCharger(void);
void MOWER_updateBladeState(Etat);
void MOWER_getCoordinates(float*, float*);
int MOWER_getAngleFromNorth(void);
float MOWER_getAzimut(float);
void MOWER_getAnglePitchRoll(double*, double*);
void MOWER_wireDetectOnLeft(void);
void MOWER_wireDetectOnRight(void);
void MOWER_wireDetectOnCharge(void);
void MOWER_sonarDetect(void);
void MOWER_bumperDetect(Bumper);
void MOWER_tiltProtection(void);
uint8_t MOWER_myRandDeg(int);
void myDelayLoop(double);

#endif /* mower_h */
