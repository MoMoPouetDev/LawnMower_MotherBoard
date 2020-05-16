//
//  mower.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 06/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <math.h>

#include "constant.h"
#include "mower.h"
#include "pwm.h"
#include "adc.h"
#include "status.h"
#include "twi.h"

void MOWER_startMower()
{
	uint8_t distanceSonarFC = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FC);
	if(distanceSonarFC == ERROR_DATA)
		distanceSonarFC = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FC);
	uint8_t distanceSonarFL = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FL);
	if(distanceSonarFL == ERROR_DATA)
		distanceSonarFL = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FL);
	uint8_t distanceSonarFR = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FR);
	if(distanceSonarFR == ERROR_DATA)
		distanceSonarFR = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FR);
			
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
		else
			PWM_forward(MIDDLE_SPEED);
    }
    else
        PWM_forward(HIGH_SPEED);
}

uint8_t isDocking()
{
	uint8_t dock = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_DOCK);
	if(dock == ERROR_DATA)
		dock = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_DOCK);
	
    return dock;
}

uint8_t isCharging()
{
	uint8_t charge = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_A);
	if(charge == ERROR_DATA)
		charge = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_A);
	
    return charge;
}

uint8_t isTimeToMow()
{
	uint8_t timeToMow = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_TIME_TO_MOW);
	if(timeToMow == ERROR_DATA)
		timeToMow = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_TIME_TO_MOW);
	
    return timeToMow;
}

uint8_t isEnoughCharged()
{
	uint8_t battery = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_V);
	if(battery == ERROR_DATA)
		battery = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_V);
	
	_uBattery = battery;
	
	if (battery <= SENSOR_V_FAIBLE_WARN) {
		if(battery <= SENSOR_V_FAIBLE_WARN)
			_eErrorMower = LOW_BATTERY;
		else if (battery <= SENSOR_V_FAIBLE_ERR)
			_eErrorMower = VERY_LOW_BATTERY;
		else if (battery <= SENSOR_V_EMPTY) {
			_eErrorMower = EMPTY_BATTERY;
			return -1;
		}
		else
			_eErrorMower = VERY_LOW_BATTERY;
		
		return 0;
	}
	else
		return 1;
}

uint8_t isRaining()
{
	uint8_t underTheRain = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_RAIN);
	if(underTheRain == ERROR_DATA)
		underTheRain = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SENSOR_RAIN);
	
	if(underTheRain) {
		_eErrorMower = DETECTED_RAIN;
		_eEtatRain = ON;
	}
	
    return underTheRain;
}

void MOWER_goDockCharger()
{
	uint8_t lastError = 0;
	uint8_t wireReached = 0;
	uint8_t distanceSonarFC,
			distanceSonarFL,
			distanceSonarFR;
	
	MOWER_directionFromBase();
	
	while(!isDocking()) {
		STATUS_updateStatus();
		STATUS_sendStatus();
		
		if(isEnoughCharged() == -1)
			break;
		
		if(!wireReached) {
			distanceSonarFC = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FC);
			if(distanceSonarFC == ERROR_DATA)
				distanceSonarFC = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FC);
			distanceSonarFL = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FL);
			if(distanceSonarFL == ERROR_DATA)
				distanceSonarFL = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FL);
			distanceSonarFR = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FR);
			if(distanceSonarFR == ERROR_DATA)
				distanceSonarFR = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FR);
			
			PWM_forward(HIGH_SPEED);
			
			if( ADC_read(PIN_ADC0) > WIRE_DETECTION_LIMITE) {
				wireReached = 1;
			}
			else if (ADC_read(PIN_ADC1) > WIRE_DETECTION_LIMITE) {
				PWM_stop();
				myDelayLoop(1000);
				PWM_reverse(LOW_SPEED);
				myDelayLoop(2000); 
				PWM_stop();
				myDelayLoop(1000);
				PWM_right();
				while(!45 || (ADC_read(PIN_ADC1) > WIRE_DETECTION_LIMITE) || (ADC_read(PIN_ADC0) > WIRE_DETECTION_LIMITE)); // Use Compass when implement, +45°
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
				MOWER_pidController(&lastError);
			}
			else {
				MOWER_directionFromBase();
				wireReached = 0;
			}
		}
	}
}

void MOWER_directionFromBase() {
	float angleFromNorth,
			angleFromBase;
	
	PWM_stop();
	myDelayLoop(1000);
	angleFromNorth = MOWER_getAngleFromNorth();
	angleFromBase = MOWER_getAzimut(angleFromNorth);
	
	if(angleFromBase < 0) {
		while(angleFromBase >= 0 ) {
			PWM_left();
			angleFromNorth = MOWER_getAngleFromNorth();
			angleFromBase = MOWER_getAzimut(angleFromNorth);
		}
	}
	else if(angleFromBase > 0) {
		while(angleFromBase <= 0) {
			PWM_right();
			angleFromNorth = MOWER_getAngleFromNorth();
			angleFromBase = MOWER_getAzimut(angleFromNorth);
		}
	}

	PWM_stop();
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

void MOWER_getCoordinates(float* pLatitudeCoordinates, float* pLongitudeCoordinates) {
    
    Coordinates tLatitude;
    Coordinates tLongitude;
    uint32_t tempLatDecimal;
    uint32_t tempLongDecimal;
    
    char tempLat[9] = {0};
    char tempLong[9] = {0};
		
	tLatitude.degrees = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEG);
	tLatitude.minutes = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_MIN);
	tLatitude.decimalMSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_MSB);
    tLatitude.decimalB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_B);
    tLatitude.decimalLSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_LSB);
    
    tempLatDecimal = ((uint32_t)tLatitude.decimalMSB << 16) | ((uint32_t)tLatitude.decimalB << 8) | ((uint32_t)tLatitude.decimalLSB);
    sprintf(tempLat, "%d.%d",(int)tLatitude.minutes, (int)tempLatDecimal);
    *pLatitudeCoordinates = (float)tLatitude.degrees + (atof(tempLat)/60.0);
    
	tLongitude.degrees = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEG);
	tLongitude.minutes = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_MIN);
	tLongitude.decimalMSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_MSB);
    tLongitude.decimalB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_B);
    tLongitude.decimalLSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_LSB);
    
    tempLongDecimal = ((uint32_t)tLongitude.decimalMSB << 16) | ((uint32_t)tLongitude.decimalB << 8) | ((uint32_t)tLongitude.decimalLSB);
    sprintf(tempLong, "%d.%d",(int)tLongitude.minutes, (int)tempLongDecimal);
    *pLongitudeCoordinates = (float)tLongitude.degrees + (atof(tempLong)/60.0);
}

float MOWER_getAngleFromNorth() {
	int8_t dataLsbX,
			dataMsbX,
			dataLsbY,
			dataMsbY;
	int16_t dataX,
			dataY;
	float angle;
			
			
	dataLsbX = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_X_LSB);
	dataMsbX = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_X_MSB);
	dataX = (dataMsbX<<8) | dataLsbX;
	
	dataLsbY = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_Y_LSB);
	dataMsbY = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_Y_MSB);
	dataY = (dataMsbY<<8) | dataLsbY;
	
	angle = atan(dataX / dataY);
	
	return angle;
}

float MOWER_getAzimut(float angleFromNorth) {
	float x,
		y,
		latitude,
		longitude,
        angle;

	MOWER_getCoordinates(&latitude, &longitude);
	
	x = cos(latitude)*sin(COORDINATES_BASE_LAT) - sin(latitude)*cos(COORDINATES_BASE_LAT)*cos(COORDINATES_BASE_LONG-longitude);
	y = sin(COORDINATES_BASE_LONG-longitude)*cos(COORDINATES_BASE_LAT);
	
	angle = 2*atan(y / (sqrt(x*x + y*y) + x));
	
	angle = angle - angleFromNorth;
	
	return angle;
}

void myDelayLoop(double delay)
{
    double i;
    for (i=0; i<delay; i++) {
        _delay_ms(1);
    }
}


