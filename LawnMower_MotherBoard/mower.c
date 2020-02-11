//
//  mower.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 06/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <util/delay.h>

#include "constant.h"
#include "mower.h"
#include "pwm.h"
#include "adc.h"

uint8_t isDocking()
{
    if ((distanceSonarFC <= SONAR_DOCKING) && (distanceSonarFL <= SONAR_DOCKING) && (distanceSonarFR <= SONAR_DOCKING))
    {
        return 1;
    }

    else
        return 0;
}

uint8_t isCharging()
{
    return inCharge;
}

uint8_t isTimeToMow()
{
    
    return 1;
}

uint8_t isEnoughCharged()
{
    if (batteryLevel <= SENSOR_V_FAIBLE_WARN) {
        return 0;
    }
    else
        return 1;
}

uint8_t isRaining()
{
    return underTheRain;
}

void startMower()
{
    if( ADC_read(PIN_ADC0) > WIRE_DETECTION_LIMITE)
    {
        PWM_stop();
        myDelayLoop(1000);
        PWM_right();
        myDelayLoop(2000); // Use Compass when implement, +135°
        PWM_stop();
        myDelayLoop(1000);
        PWM_forward(MIDDLE_SPEED);
        myDelayLoop(1000);
    }
    else if (ADC_read(PIN_ADC1) > WIRE_DETECTION_LIMITE)
    {
        PWM_stop();
        myDelayLoop(1000);
        PWM_left();
        myDelayLoop(2000); // Use Compass when implement, -135°
        PWM_stop();
        myDelayLoop(1000);
        PWM_forward(MIDDLE_SPEED);
        myDelayLoop(1000);
    }
    else if ((distanceSonarFC < SONAR_WARN) || (distanceSonarFL < SONAR_WARN) || (distanceSonarFR < SONAR_WARN))
    {
        PWM_forward(MIDDLE_SPEED);
        
        if ((distanceSonarFC < SONAR_ERR) || (distanceSonarFL < SONAR_ERR) || (distanceSonarFR < SONAR_ERR)) {
            PWM_stop();
            myDelayLoop(1000);
            PWM_right();
            myDelayLoop(2000); // Use Compass when implement, +45°
            PWM_stop();
            myDelayLoop(1000);
            PWM_forward(MIDDLE_SPEED);
            myDelayLoop(1000);
        }
    }
    else
        PWM_forward(HIGH_SPEED);
}

void goDockCharger()
{
    
}

void leaveDockCharger()
{
    PWM_reverse(LOW_SPEED);
    myDelayLoop(5000);
    PWM_stop();
    myDelayLoop(1000);
    
    if (isRaining()) {
        PWM_forward(LOW_SPEED);
        while (!(isDocking()));
        PWM_stop();
    }
    else
    {
        PWM_right();
        myDelayLoop(2000); // Use Compass when implement, +45°
        PWM_stop();
        PWM_forward(MIDDLE_SPEED);
        myDelayLoop(1000);
    }
}

void updateBladeState()
{
    if (etatBlade) {
        PORTB |= (1<<PORTB6) | (1<<PORTB7);
    }
    else if (!etatBlade)
    {
        PORTB &= ~(1<<PORTB6) & ~(1<<PORTB7);
    }
}

void myDelayLoop(double delay)
{
    double i;
    for (i=0; i<delay; i++) {
        _delay_ms(1);
    }
}


