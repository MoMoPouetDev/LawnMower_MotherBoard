//
//  pwm.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 30/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef pwm_h
#define pwm_h

void PWM_forward(uint8_t);
void PWM_forward_turn(uint8_t, uint8_t);
void PWM_reverse(uint8_t);
void PWM_right(void);
void PWM_left(void);
void PWM_stop(void);


#endif /* pwm_h */
