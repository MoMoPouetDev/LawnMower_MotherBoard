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
uint8_t isEnoughCharged(void);
uint8_t isRaining(void);
void startMower(void);
void goDockCharger(void);
void leaveDockCharger(void);
void updateBladeState(void);
void myDelayLoop(double);

#endif /* mower_h */