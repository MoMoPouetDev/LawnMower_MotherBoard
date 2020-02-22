//
//  mower.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 06/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef mower_h
#define mower_h

uint8_t isDocking(void);
uint8_t isCharging(void);
uint8_t isTimeToMow(void);
uint8_t isEnoughCharged(uint8_t);
uint8_t isRaining(void);
void MOWER_startMower(void);
void MOWER_goDockCharger(void);
void MOWER_directionFromBase(void);
void MOWER_pidController(uint8_t*);
void MOWER_leaveDockCharger(void);
void MOWER_updateBladeState(void);
void myDelayLoop(double);

#endif /* mower_h */