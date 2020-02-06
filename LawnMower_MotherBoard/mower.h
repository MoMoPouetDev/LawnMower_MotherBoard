//
//  mower.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 06/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef mower_h
#define mower_h

#include <stdio.h>

int isDocking(void);
int isCharging(void);
int isTimeToMow(void);
int isEnoughCharged(void);
int isRaining(void);
void startMower(void);
void goDockCharger(void);
void leaveDockCharger(void);

#endif /* mower_h */
