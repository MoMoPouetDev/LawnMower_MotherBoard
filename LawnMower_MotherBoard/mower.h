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
    uint8_t decimal;
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
uint8_t MOWER_leaveDockCharger(void);
void MOWER_updateBladeState(void);
void MOWER_getCoordinates(float*, float*);
float MOWER_getAngleFromNorth(void);
float MOWER_getAzimut(float);
void myDelayLoop(double);

#endif /* mower_h */
