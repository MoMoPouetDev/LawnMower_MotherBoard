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
#include "status.h"

void MOWER_startMower()
{
    if( ADC_read(PIN_ADC0) > WIRE_DETECTION_LIMITE)
    {
        PWM_stop();
        myDelayLoop(1000);
        PWM_right();
        myDelayLoop(2000); // Use Compass when implement, +135°
        PWM_stop();
        myDelayLoop(1000);
        PWM_forward(LOW_SPEED);
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
        PWM_forward(LOW_SPEED);
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
            PWM_forward(LOW_SPEED);
            myDelayLoop(1000);
        }
    }
    else
        PWM_forward(HIGH_SPEED);
}

uint8_t isDocking()
{
    return _uDock;
}

uint8_t isCharging()
{
    return _uCharge;
}

uint8_t isTimeToMow()
{
    return _uTimeToMow;
}

uint8_t isEnoughCharged(uint8_t dock)
{
	if(dock) {
		if (_uBattery <= SENSOR_V_OK) {
			return 0;
		}
		else
			return 1;
	}
	else {
		if (_uBattery <= SENSOR_V_FAIBLE_WARN) {
			if(_uBattery <= SENSOR_V_FAIBLE_WARN)
				_eErrorMower = LOW_BATTERY;
			else if (_uBattery <= SENSOR_V_FAIBLE_ERR)
				_eErrorMower = VERY_LOW_BATTERY;
			else if (_uBattery <= SENSOR_V_EMPTY)
				_eErrorMower = EMPTY_BATTERY;
			else
				_eErrorMower = VERY_LOW_BATTERY;
			
			return 0;
		}
		else
			return 1;
	}
}

uint8_t isRaining()
{
	if(_uUnderTheRain) {
		_eErrorMower = DETECTED_RAIN;
		_eEtatRain = ON;
	}
    return _uUnderTheRain;
}

void MOWER_goDockCharger()
{
	uint8_t _pLastError = 0;
	uint8_t _uChargeStatus = 0;
	
	MOWER_directionFromBase();
	
	while(!isDocking()) {
		_uChargeStatus = isEnoughCharged(isDocking());
		STATUS_sendStatus();
		
		if(!_uWireReached) {
			PWM_forward(HIGH_SPEED);
			
			if( ADC_read(PIN_ADC0) > WIRE_DETECTION_LIMITE) {
				_uWireReached = 1;
			}
			else if (ADC_read(PIN_ADC1) > WIRE_DETECTION_LIMITE) {
				PWM_stop();
				myDelayLoop(1000);
				PWM_reverse(LOW_SPEED);
				myDelayLoop(2000); 
				PWM_stop();
				myDelayLoop(1000);
				PWM_right();
				while(!45 || (ADC_read(PIN_ADC1) > WIRE_DETECTION_LIMITE)); // Use Compass when implement, +45°
			}
			else if ((distanceSonarFC < SONAR_WARN) || (distanceSonarFL < SONAR_WARN) || (distanceSonarFR < SONAR_WARN)) {
				PWM_forward(MIDDLE_SPEED);
        
				if ((distanceSonarFC < SONAR_ERR) || (distanceSonarFL < SONAR_ERR) || (distanceSonarFR < SONAR_ERR)) {
					PWM_stop();
					myDelayLoop(1000);
					PWM_right();
					myDelayLoop(2000); // Use Compass when implement, +45°
					PWM_stop();
					myDelayLoop(1000);
					PWM_forward(LOW_SPEED);
					myDelayLoop(1000);
					MOWER_directionFromBase();
				}
			}
		}
		else {
			if( (WIRE_DETECTION_MIN < ADC_read(PIN_ADC0)) && (ADC_read(PIN_ADC0 < WIRE_DETECTION_MAX))) {
				MOWER_pidController(&_pLastError);
			}
			else {
				MOWER_directionFromBase();
				_uWireReached = 0;
			}
		}
	}
}

void MOWER_directionFromBase() {
	PWM_stop();
	myDelayLoop(1000);
/*	getAbsoluteAngle();
	getAngleFromBase();
	if(-) {
		PWM_left();
	}
	else if(+) {
		PWM_right();
	}
*/
}

void MOWER_pidController(uint8_t* lastError) {
	uint8_t Kp = 2,
			Kd = 1,
			currentPosition = 0;
			
	int errorPosition = 0,
		derivativePosition = 0,
		wirePwm = 0;
	
	currentPosition = ADC_read(PIN_ADC0);
	errorPosition = WIRE_DETECTION_MAX - currentPosition;
	derivativePosition = errorPosition - *lastError;
	wirePwm = ((((Kp*errorPosition) + (Kd*derivativePosition))/10)/2);
	
	if(wirePwm > 50) {
		wirePwm = 50;
	}
	else if(wirePwm < -50) {
		wirePwm = -50;
	}
	
	if(wirePwm > 0) {
		wirePwm = (50-wirePwm);
		if(wirePwm < 10)
			wirePwm = LOW_SPEED;
		PWM_forward_turn(wirePwm, MIDDLE_SPEED);
	}
	else if(wirePwm < 0) {
		wirePwm = (wirePwm-50);
		if(wirePwm > -10)
			wirePwm = LOW_SPEED;
		else
			wirePwm = -wirePwm;
		PWM_forward_turn(MIDDLE_SPEED, wirePwm);
	}
	else
		PWM_forward(MIDDLE_SPEED);
	
	*lastError = errorPosition;
}

void MOWER_leaveDockCharger()
{
    PWM_reverse(LOW_SPEED);
    myDelayLoop(5000);
    PWM_stop();
    myDelayLoop(1000);
    PWM_right();
	myDelayLoop(2000); // Use Compass when implement, +45°
    PWM_stop();
    PWM_forward(MIDDLE_SPEED);
    myDelayLoop(1000);
}

void MOWER_updateBladeState()
{
	switch(_eEtatBlade) {
		case ON:
			PORTB |= (1<<PORTB6) | (1<<PORTB7);
			break;
		case OFF:
			PORTB &= ~(1<<PORTB6) & ~(1<<PORTB7);
			break;
		default:
			PORTB &= ~(1<<PORTB6) & ~(1<<PORTB7);
			break;
	}
}

void myDelayLoop(double delay)
{
    double i;
    for (i=0; i<delay; i++) {
        _delay_ms(1);
    }
}


