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
#include <avr/wdt.h>

#include "adc.h"
#include "constant.h"
#include "mower.h"
#include "pwm.h"
#include "status.h"
#include "twi.h"
#include "fifo.h"

void MOWER_startMower()
{

	static uint8_t uDistanceSonarFC = 255,
					uDistanceSonarFL = 255,
					uDistanceSonarFR = 255;
	static uint16_t uDistanceWireLeft = WIRE_DETECTION_UNLOAD,
					uDistanceWireRight = WIRE_DETECTION_UNLOAD;
	
	MOWER_getSonarDistance(&uDistanceSonarFC, &uDistanceSonarFL, &uDistanceSonarFR);
	MOWER_getWireDistanceLeft(&uDistanceWireLeft);
	MOWER_getWireDistanceRight(&uDistanceWireRight);
	
	MOWER_tiltProtection();
	
	if (uDistanceWireLeft > WIRE_DETECTION_LIMITE) {
		MOWER_wireDetectOnLeft(&uDistanceWireLeft);
		PWM_forward(MIDDLE_SPEED);
	}
	else if (uDistanceWireRight > WIRE_DETECTION_LIMITE) {
		MOWER_wireDetectOnRight(&uDistanceWireRight);
		PWM_forward(MIDDLE_SPEED);
	}
	else if ((uDistanceSonarFC < SONAR_WARN) || (uDistanceSonarFL < SONAR_WARN) || (uDistanceSonarFR < SONAR_WARN))
	{
		PWM_forward(MIDDLE_SPEED);
		/***** 
		if ((uDistanceSonarFC < SONAR_ERR) || (uDistanceSonarFL < SONAR_ERR) || (uDistanceSonarFR < SONAR_ERR)) {
			MOWER_sonarDetect(&uDistanceSonarFC, &uDistanceSonarFL, &uDistanceSonarFR);
			PWM_forward(MIDDLE_SPEED);
		}
		else {
			PWM_forward(MIDDLE_SPEED);
		}
		*****/
	}
	else {
		PWM_forward(HIGH_SPEED);
	}
}

uint8_t isDocking()
{	
    return _uDock;
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
	uint8_t underTheRain = 0;
	
	if(underTheRain) {
		_eErrorMower = DETECTED_RAIN;
		_eEtatRain = ON;
	}
	
    return underTheRain;
}

void MOWER_goDockCharger()
{
	static uint8_t uWireReached = 0,
					uFlagDirectionFromBase = 0;
	
	static uint8_t uDistanceSonarFC = 255,
					uDistanceSonarFL = 255,
					uDistanceSonarFR = 255;
					
	static uint16_t uDistanceWireLeft = WIRE_DETECTION_UNLOAD,
					uDistanceWireRight = WIRE_DETECTION_UNLOAD;
	
	MOWER_getSonarDistance(&uDistanceSonarFC, &uDistanceSonarFL, &uDistanceSonarFR);
	MOWER_getWireDistanceLeft(&uDistanceWireLeft);
	MOWER_getWireDistanceRight(&uDistanceWireRight);
	
	if (!uFlagDirectionFromBase) {
		MOWER_directionFromBase();
		uFlagDirectionFromBase = 1;
	}
	
	MOWER_tiltProtection();
		
	if(!uWireReached) {		
		if (uDistanceWireLeft > WIRE_DETECTION_LIMITE) {
			uWireReached = 1;
		}
		else if (uDistanceWireRight > WIRE_DETECTION_LIMITE) {
			MOWER_wireDetectOnCharge();
			PWM_forward(MIDDLE_SPEED);
		}
		else if ((uDistanceSonarFC < SONAR_WARN) || (uDistanceSonarFL < SONAR_WARN) || (uDistanceSonarFR < SONAR_WARN)) {
			PWM_forward(MIDDLE_SPEED);
		}
		else {
			PWM_forward(HIGH_SPEED);
		}
	}
	else {
		if ((WIRE_DETECTION_UNLOAD < uDistanceWireLeft) && (uDistanceWireLeft < WIRE_DETECTION_LOAD)) {
			MOWER_pidController();
		}
		else {
			uFlagDirectionFromBase = 0;
			uWireReached = 0;
		}
	}
}

void MOWER_directionFromBase() {
	uint8_t deltaAngle = DELTA_ANGLE;
	uint16_t angleFromNorth = MOWER_getAngleFromNorth();
	uint16_t currentAngle;
	uint16_t angleFromBase = MOWER_getAzimut(angleFromNorth);
	
	uint16_t uDistanceWireLeft = WIRE_DETECTION_UNLOAD;
	uint16_t uDistanceWireRight = WIRE_DETECTION_UNLOAD;

	PWM_stop();
	myDelayLoop(1);
	PWM_right();
	
	while (!((currentAngle > ((angleFromBase - deltaAngle)%360)) && (currentAngle < ((angleFromBase + deltaAngle)%360)))) {
		wdt_reset();
		MOWER_tiltProtection();
		
		MOWER_getWireDistanceLeft(&uDistanceWireLeft);
		MOWER_getWireDistanceRight(&uDistanceWireRight);
		if ((uDistanceWireLeft > WIRE_DETECTION_LIMITE) || (uDistanceWireRight > WIRE_DETECTION_LIMITE)) {
			break;
		}
	}
	PWM_stop();
}

void MOWER_pidController() {
	uint8_t Kp = 0.2;
	uint16_t currentPosition = 0;
			
	int errorPosition = 0,
		wirePwm = 0;
	
	MOWER_wireDetectOnLeft(&currentPosition);
	errorPosition = WIRE_DETECTION_LOAD - currentPosition;
	wirePwm = (Kp*errorPosition);
	
	if(wirePwm > MIDDLE_SPEED) {
		wirePwm = MIDDLE_SPEED;
	}
	else if(wirePwm < -MIDDLE_SPEED) {
		wirePwm = -MIDDLE_SPEED;
	}
	
	if(wirePwm > 0) {
		PWM_forward_turn(MIDDLE_SPEED - wirePwm, MIDDLE_SPEED);
	}
	else if(wirePwm < 0) {
		PWM_forward_turn(MIDDLE_SPEED, MIDDLE_SPEED - wirePwm);
	}
	else {
		PWM_forward(MIDDLE_SPEED);
	}	
}

void MOWER_leaveDockCharger()
{
	uint8_t deltaAngle = DELTA_ANGLE;
	//volatile uint8_t randAngle = MOWER_myRandDeg(8);
	uint16_t randAngle = MOWER_myRandDeg(180);
	uint16_t startAngle = MOWER_getAngleFromNorth();
	uint16_t currentAngle;
	uint16_t endAngle = (startAngle + randAngle)%180;
	
	uint16_t uDistanceWireRight = WIRE_DETECTION_UNLOAD;
	
	PWM_reverse(MIDDLE_SPEED);
    myDelayLoop(5);
    PWM_stop();
    myDelayLoop(1);
    PWM_right();
	
    while( !((currentAngle > ((endAngle - deltaAngle)%180)) && (currentAngle < ((endAngle + deltaAngle)%180))) ) {
	    wdt_reset();
	    currentAngle = MOWER_getAngleFromNorth();
	    MOWER_tiltProtection();
		
		MOWER_wireDetectOnRight(&uDistanceWireRight);
		if (uDistanceWireRight > WIRE_DETECTION_LIMITE) {
			break;
		}
    }
    
	PWM_stop();
}

void MOWER_updateBladeState(Etat eEtatBlade)
{
	switch(eEtatBlade) {
		case ON:
			PORTB |= (1<<PORTB5);
			break;
		case OFF:
			PORTB &= ~(1<<PORTB5);
			break;
		default:
			PORTB &= ~(1<<PORTB5);
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

int MOWER_getAngleFromNorth() {
	uint8_t dataLsbX,
			dataMsbX,
			dataLsbY,
			dataMsbY,
            dataLsbZ,
            dataMsbZ;
	static int dataX = 0,
        dataY = 0,
        dataZ = 0,
		angle = 0;
	double dPitch,
			dRoll;
	float xh,
		yh,
		rPitch,
		rRoll;
			
    dataLsbX = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_X_LSB);
    dataMsbX = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_X_MSB);
    dataX = (int)(int16_t)((dataMsbX<<8) | dataLsbX);
	
	dataLsbY = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_Y_LSB);
	dataMsbY = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_Y_MSB);
    dataY = (int)(int16_t)((dataMsbY<<8) | dataLsbY);
    
    dataLsbZ = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_Z_LSB);
    dataMsbZ = TWI_getData(ADDR_SLAVE_COMPASS, ADDR_DATA_COMPASS_Z_MSB);
    dataZ = (int)(int16_t)((dataMsbZ<<8) | dataLsbZ);
        
	/*** Without Tilt Compensation ***
    angle = (atan2(-(dataY-((CALIBRATION_Y_MAX + CALIBRATION_Y_MIN)/2.0)),(dataX - ((CALIBRATION_X_MAX + CALIBRATION_X_MIN)/2.0)))) + DECLINATION + OFFSET;
    
	if (angle > (2*M_PI))
	angle = angle - (2*M_PI);
	if (angle < 0)
	angle = angle + (2*M_PI);
	
	return (angle * (180/M_PI));	
	/*** END ***/
	
	/*** With Tilt Compensation ***/
	MOWER_getAnglePitchRoll(&dPitch, &dRoll);
	
	rPitch = (M_PI/180)*dPitch;
	rRoll = (M_PI/180)*dRoll;
  
	xh = ((float)(dataX - OFFSET_X) * cos(rPitch)) + ((float)(dataY - OFFSET_Y) * sin(rRoll) * sin(rPitch)) - ((float)(dataZ - OFFSET_Z) * cos(rRoll) * sin(rPitch));
	yh = ((float)(dataY - OFFSET_Y) * cos(rRoll)) + ((float)(dataZ - OFFSET_Z) * sin(rRoll));
  
	angle = (180/M_PI) * (atan2(-yh,xh)+ DECLINATION + OFFSET);
  
	return (int)(-angle + 360) % 360;
	/*** END ***/
}

int MOWER_getAzimut(float angleFromNorth) {
	float x,
		y,
		latitude,
		longitude,
        angle;

	MOWER_getCoordinates(&latitude, &longitude);
	
	x = cos(latitude)*sin(COORDINATES_BASE_LAT) - sin(latitude)*cos(COORDINATES_BASE_LAT)*cos(COORDINATES_BASE_LONG-longitude);
	y = sin(COORDINATES_BASE_LONG-longitude)*cos(COORDINATES_BASE_LAT);
	
	angle = 2*atan(y / (sqrt(x*x + y*y) + x));
	
	//angle = angle - angleFromNorth;
	
	return (int)angle;
}

void MOWER_getAnglePitchRoll(double* pPitch, double* pRoll) {
	uint8_t dataLsbX,
		dataMsbX,
		dataLsbY,
		dataMsbY,
		dataLsbZ,
		dataMsbZ;
	static float   dataX = 0,
                    dataY = 0,
                    dataZ = 0;
	double valuePitch,
			valueRoll;
    
    dataLsbX = TWI_getData(ADDR_SLAVE_ACCELEROMETER, ADDR_DATA_ACCELEROMETER_X_LSB);
    dataMsbX = TWI_getData(ADDR_SLAVE_ACCELEROMETER, ADDR_DATA_ACCELEROMETER_X_MSB);
    dataX = ((float)(int16_t)((dataMsbX<<8) | dataLsbX))/256;
    
    dataLsbY = TWI_getData(ADDR_SLAVE_ACCELEROMETER, ADDR_DATA_ACCELEROMETER_Y_LSB);
    dataMsbY = TWI_getData(ADDR_SLAVE_ACCELEROMETER, ADDR_DATA_ACCELEROMETER_Y_MSB);
    dataY = ((float)(int16_t)((dataMsbY<<8) | dataLsbY))/256;
    
    dataLsbZ = TWI_getData(ADDR_SLAVE_ACCELEROMETER, ADDR_DATA_ACCELEROMETER_Z_LSB);
    dataMsbZ = TWI_getData(ADDR_SLAVE_ACCELEROMETER, ADDR_DATA_ACCELEROMETER_Z_MSB);
    dataZ = ((float)(int16_t)((dataMsbZ<<8) | dataLsbZ))/256;
	
    valuePitch = 180 * atan2(-dataX, sqrt(dataY*dataY + dataZ*dataZ))/M_PI;
    valueRoll = 180 * atan2(dataY, sqrt(dataX*dataX + dataZ*dataZ))/M_PI;
	
	*pPitch = (double)FIFO_getAverage(&fifoPitch, (int)valuePitch);
	*pRoll = (double)FIFO_getAverage(&fifoRoll, (int)valueRoll);
}

void MOWER_wireDetectOnLeft(uint16_t* pDistanceWireLeft){
	uint8_t deltaAngle = DELTA_ANGLE;
	uint16_t randAngle = MOWER_myRandDeg(360);
	uint16_t startAngle = MOWER_getAngleFromNorth();
	uint16_t currentAngle;
	uint16_t endAngle = (startAngle + randAngle)%360;
	
	uint16_t uDistanceWireRight = WIRE_DETECTION_UNLOAD;
	
	PWM_stop();
	myDelayLoop(1);
	
	PWM_reverse(MIDDLE_SPEED);
	
	while(*pDistanceWireLeft > WIRE_DETECTION_LIMITE) {
		wdt_reset();
		MOWER_getWireDistanceLeft(pDistanceWireLeft);
		MOWER_tiltProtection();
	}
	
	PWM_stop();
	myDelayLoop(1);
	
	PWM_right();

	while( !((currentAngle > ((endAngle - deltaAngle)%360)) && (currentAngle < ((endAngle + deltaAngle)%360))) ) {
		wdt_reset();
		currentAngle = MOWER_getAngleFromNorth();
		MOWER_tiltProtection();
		
		MOWER_getWireDistanceRight(&uDistanceWireRight);
		if (uDistanceWireRight > WIRE_DETECTION_LIMITE) {
			break;
		}
	}
	
	PWM_stop();
	myDelayLoop(1);
}

void MOWER_wireDetectOnRight(uint16_t* pDistanceWireRight){
	uint8_t deltaAngle = DELTA_ANGLE;
	uint16_t randAngle = MOWER_myRandDeg(360);
	uint16_t startAngle = MOWER_getAngleFromNorth();
	uint16_t currentAngle;
	uint16_t endAngle = (startAngle + randAngle)%360;
	
	uint16_t uDistanceWireLeft = WIRE_DETECTION_UNLOAD;
	
	PWM_stop();
	myDelayLoop(1);
	
	PWM_reverse(MIDDLE_SPEED);
	while(*pDistanceWireRight > WIRE_DETECTION_LIMITE) {
		wdt_reset();
		MOWER_getWireDistanceRight(pDistanceWireRight);
		MOWER_tiltProtection();
	}
	
	PWM_stop();
	myDelayLoop(1);
	
	PWM_left();

	while( !((currentAngle > ((endAngle - deltaAngle)%360)) && (currentAngle < ((endAngle + deltaAngle)%360))) ) {
		wdt_reset();
		currentAngle = MOWER_getAngleFromNorth();
		MOWER_tiltProtection();
		
		MOWER_wireDetectOnLeft(&uDistanceWireLeft);
		if (uDistanceWireLeft > WIRE_DETECTION_LIMITE) {
			break;
		}
	}

	PWM_stop();
	myDelayLoop(1);
}

void MOWER_wireDetectOnCharge(){
    uint8_t deltaAngle = DELTA_ANGLE;
    uint8_t nextAngle = 90;
    uint16_t startAngle = MOWER_getAngleFromNorth();
    uint16_t endAngle = (startAngle + nextAngle)%360;
	
	uint16_t uDistanceWireLeft = WIRE_DETECTION_UNLOAD;
    
    PWM_stop();
    myDelayLoop(1);
	
	PWM_left();
	
    MOWER_getWireDistanceLeft(&uDistanceWireLeft);
	while(uDistanceWireLeft > WIRE_DETECTION_LIMITE) {
		wdt_reset();
		MOWER_getWireDistanceLeft(&uDistanceWireLeft);
		MOWER_tiltProtection();
	}
	
    PWM_stop();
}

void MOWER_sonarDetect(uint8_t* pDistanceSonarFC, uint8_t* pDistanceSonarFL, uint8_t* pDistanceSonarFR) {
	uint8_t deltaAngle = DELTA_ANGLE;
	uint16_t randAngle = MOWER_myRandDeg(360);
	uint16_t startAngle = MOWER_getAngleFromNorth();
	uint16_t currentAngle;
	uint16_t endAngle = (startAngle + randAngle)%360;
	
	PWM_stop();
	myDelayLoop(1);
	
	PWM_reverse(HIGH_SPEED);
	MOWER_getSonarDistance(pDistanceSonarFC, pDistanceSonarFL, pDistanceSonarFR);
	while( (*pDistanceSonarFC < SONAR_LIMITE) || (*pDistanceSonarFL < SONAR_LIMITE) || (*pDistanceSonarFR < SONAR_LIMITE) ) {
		wdt_reset();
		MOWER_getSonarDistance(pDistanceSonarFC, pDistanceSonarFL, pDistanceSonarFR);
		MOWER_tiltProtection();
	}
	
	PWM_stop();
	myDelayLoop(1);
	
	PWM_right();
	
	currentAngle = MOWER_getAngleFromNorth();
	while( !((currentAngle > ((endAngle - deltaAngle)%360)) && (currentAngle < ((endAngle + deltaAngle)%360))) ) {
		wdt_reset();
		currentAngle = MOWER_getAngleFromNorth();
		MOWER_tiltProtection();
	}

	PWM_stop();
	myDelayLoop(1);
}

void MOWER_bumperDetect(Bumper _bumper) {
	
	uint8_t deltaAngle = DELTA_ANGLE;
	uint16_t randAngle = MOWER_myRandDeg(360);
	uint16_t startAngle = MOWER_getAngleFromNorth();
	uint16_t currentAngle;
	uint16_t endAngle = (startAngle + randAngle)%360;
	 
	PWM_stop();
	myDelayLoop(1);
	 
	PWM_reverse(MIDDLE_SPEED);
	myDelayLoop(1);
	if(_bumper == FL)
		while( !(PINB & (1<<PINB1)) );
	if(_bumper == FC)
		while( !(PINB & (1<<PINB2)) );
	if(_bumper == FR)
		while( !(PINB & (1<<PINB4)) );
	 
	 PWM_stop();
	 myDelayLoop(1);
	 
	 PWM_right();
	 currentAngle = MOWER_getAngleFromNorth();
	 while( !((currentAngle > ((endAngle - deltaAngle)%360)) && (currentAngle < ((endAngle + deltaAngle)%360))) ) {
		 wdt_reset();
		 currentAngle = MOWER_getAngleFromNorth();
		 MOWER_tiltProtection();
	 }
	 
	 PWM_stop();
	 myDelayLoop(1);
	
}

void MOWER_tiltProtection() {
	double dPitch,
			dRoll;
			
	MOWER_getAnglePitchRoll(&dPitch, &dRoll);
	
	if((dPitch <= PITCH_MIN) || (dPitch >= PITCH_MAX) || (dRoll <= ROLL_MIN) || (dRoll >= ROLL_MAX)) { 
		MOWER_updateBladeState(OFF);
	}
	else {
		MOWER_updateBladeState(ON);
	}
}

void MOWER_getSonarDistance(uint8_t* pDistanceSonarFC, uint8_t* pDistanceSonarFL, uint8_t* pDistanceSonarFR) {
	uint8_t valueFC = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FC);
	*pDistanceSonarFC = (uint8_t)FIFO_getAverage(&fifoSonarFC, (int)valueFC);
	
	uint8_t valueFL = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FL);
	*pDistanceSonarFL = (uint8_t)FIFO_getAverage(&fifoSonarFL, (int)valueFL);
	
	uint8_t valueFR = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_SONAR_FR);
	*pDistanceSonarFR = (uint8_t)FIFO_getAverage(&fifoSonarFR, (int)valueFR);
}

void MOWER_getWireDistanceLeft(uint16_t* pDistanceWireLeft) {
	uint16_t value = ADC_read(PIN_ADC0_LS);
	*pDistanceWireLeft = (uint16_t)FIFO_getAverage(&fifoLeftWire, (int)value);
}

void MOWER_getWireDistanceRight(uint16_t* pDistanceWireRight) {
	uint16_t value = ADC_read(PIN_ADC1_RS);
	*pDistanceWireRight = (uint16_t)FIFO_getAverage(&fifoRightWire, (int)value);
}

uint8_t MOWER_myRandDeg(int modulo) {
    return (uint8_t)((rand()%modulo) + 1);
}

void myDelayLoop(uint16_t delay)
{
	uint16_t myMillSec = 1000 * delay;
	uint16_t i;
	
	TCNT1 = 0x00;
	TCCR1B |= (1<<CS10);
	
	for (i = 0; i<= myMillSec; i++)
	{
		while (TCNT1 <= 0x1F40);
		TCNT1 = 0x00;
	}
	
	TCCR1B &= ~(1<<CS10);
}

