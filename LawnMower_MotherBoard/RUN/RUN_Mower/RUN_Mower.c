/*
 * RUN_Mower.c
 *
 *  Created on: 05 MAR 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "HAL_ADC.h"
#include "HAL_I2C.h"
#include "HAL_Timer.h"
#include "HAL_GPIO.h"
#include "RUN_PWM.h"
#include "RUN_FIFO.h"
#include "RUN_Mower.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
/***  GuideWire ***/
#define WIRE_DETECTION_LIMITE 600
#define WIRE_DETECTION_LOAD 750
#define WIRE_DETECTION_UNLOAD 450
/*** Motor ***/
#define HIGH_SPEED 100
#define MIDDLE_SPEED 90
#define LOW_SPEED 80
/*** Timer ***/
#define GPT_ONE_SECOND 100
#define GPT_FIVE_SECOND 500
/*** GPS ***/
#define COORDINATES_BASE_LAT 49.2315928
#define COORDINATES_BASE_LONG 1.2470619
/*** Time to Mow ***/
#define THRESHOLD_HOUR_MIN 9
#define THRESHOLD_HOUR_MAX 18
/*** Compass ***/
#define DELTA_ANGLE 5
//#define M_PI 3.14
#define DECLINATION ((54)*(M_PI/(60*180))) //0.015
#define OFFSET (19*M_PI)/16
#define CALIBRATION_X_MAX 646
#define CALIBRATION_X_MIN 192
#define CALIBRATION_Y_MAX 602
#define CALIBRATION_Y_MIN 188
#define CALIBRATION_Z_MAX 39
#define CALIBRATION_Z_MIN -378
#define OFFSET_X ((CALIBRATION_X_MAX + CALIBRATION_X_MIN)/2)
#define OFFSET_Y ((CALIBRATION_Y_MAX + CALIBRATION_Y_MIN)/2)
#define OFFSET_Z ((CALIBRATION_Z_MAX + CALIBRATION_Z_MIN)/2)
/*** Accel ***/
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
/*** Sonar ***/
#define SONAR_WARN 30
#define SONAR_LIMITE 20
#define SONAR_ERR 10
#define SONAR_DIST_ERR 999
/*** Variables ***/
static uint8_t gu8_deltaAngle;
static uint8_t gu8_distanceFC;
static uint8_t gu8_distanceFL;
static uint8_t gu8_distanceFR;
static uint16_t gu16_distanceWireLeft;
static uint16_t gu16_distanceWireRight;
static double gd_pitch;
static double gd_roll;
static uint16_t gu16_currentAngle;
static uint16_t gu16_azimut;
static EtatMower geEtatMower;
static ErrorMower geErrorMower;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static uint16_t _RUN_Mower_MyRandDeg(uint16_t u16_modulo);
static void _RUN_Mower_GetAnglePitchRoll(double* pd_pitch, double* pd_roll, uint8_t* pu8_rxBuffAccel, uint8_t* pu8_rxBuffAccelSize);
static int16_t _RUN_Mower_GetAngleFromNorth(double d_pitch, double d_roll, uint8_t* pu8_rxBuffCompass, uint8_t* pu8_rxBuffAccelSize);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Mower_Init()
{
	gu8_deltaAngle = DELTA_ANGLE;
	gu8_distanceFC = 255;
  	gu8_distanceFL = 255;
  	gu8_distanceFR = 255;
  	gu16_distanceWireLeft = WIRE_DETECTION_UNLOAD;
  	gu16_distanceWireRight = WIRE_DETECTION_UNLOAD;
	gd_pitch = 0;
	gd_roll = 0;
	gu16_currentAngle = 0;
	gu16_azimut = 0;
}

uint8_t RUN_Mower_IsTimeToMow()
{
	uint8_t u8_returnValue = 0;
	uint8_t u8_hours = 0;

	u8_hours = 0;//RUN_Sensors_GetHoursFromGPS(); get from GPS throught I2C - MVE

	if ((THRESHOLD_HOUR_MIN <= u8_hours) && (u8_hours < THRESHOLD_HOUR_MAX))
	{
		u8_returnValue = 1;
	}

	return u8_returnValue;
}

uint8_t RUN_Mower_LeaveDockCharger()
{
	static uint8_t _u8_leaveState = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint16_t _u16_cptValue = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_leaveState)
   	{
		default:
	  	case 0:
			_u16_randAngle = _RUN_Mower_MyRandDeg(180);
			_u16_startAngle = gu16_currentAngle;
			_u8_leaveState = 1;

			break;
		case 1:
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%180;
			RUN_PWM_Backward(MIDDLE_SPEED);
			_u8_leaveState = 2;

			break;
		case 2 :
			if ( (_u16_cptValue) >= GPT_FIVE_SECOND )
			{
				RUN_PWM_Stop();
				RUN_PWM_Right();
				_u16_cptValue = 0;
				_u8_leaveState = 3;
			}
			else
			{
				_u16_cptValue++;
			}
			
			break;
		case 3 :
			if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%180)) && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%180)) )
			{
				_u8_leaveState = 4;
			}
			else
			{
				gu16_distanceWireRight = HAL_ADC_GetRightWireValue();
				if (gu16_distanceWireRight > WIRE_DETECTION_LIMITE)
				{
					_u8_leaveState = 4;
				}
			}

			break;
		case 4 :
			RUN_PWM_Stop();
			_u8_leaveState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

static uint16_t _RUN_Mower_MyRandDeg(uint16_t u16_modulo)
{
    return (uint16_t)((rand()%u16_modulo) + 1);
}

void RUN_Mower_GetAngles()
{
	static uint8_t _tu8_rxBuffCompass[6] = {0};
	static uint8_t _u8_rxBuffCompassSize = 0;
	static uint8_t _tu8_rxBuffAccel[6] = {0};
	static uint8_t _u8_rxBuffAccelSize = 0;
	static uint8_t _u8_getAngleState = 1;
	uint8_t u8_flagI2c = 0;

	switch (_u8_getAngleState)
	{
		default:
		case 0 :
			u8_flagI2c = HAL_I2C_ReadAccel(_tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
			if (u8_flagI2c)
			{
				_RUN_Mower_GetAnglePitchRoll(&gd_pitch, &gd_roll, _tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
				_u8_getAngleState = 1;
			}

			break;
		case 1 :
			u8_flagI2c = HAL_I2C_ReadCompass(_tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
			if (u8_flagI2c)
			{
				gu16_currentAngle = _RUN_Mower_GetAngleFromNorth(gd_pitch, gd_roll, _tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
				_u8_getAngleState = 0;
			}

			break;
	}
}

void RUN_Mower_GetAzimut() 
{
	float f_latitude;
	float f_longitude;
	float x = 0.0;
	float y = 0.0;


	//HAL_GPS_GetCoordinates(&f_latitude, &f_longitude); Get from slave - MVE
	
	x = cos(f_latitude)*sin(COORDINATES_BASE_LAT) - sin(f_latitude)*cos(COORDINATES_BASE_LAT)*cos(COORDINATES_BASE_LONG-f_longitude);
	y = sin(COORDINATES_BASE_LONG-f_longitude)*cos(COORDINATES_BASE_LAT);
	
	gu16_azimut = 2*atan(y / (sqrt(x*x + y*y) + x));
}

static int16_t _RUN_Mower_GetAngleFromNorth(double d_pitch, double d_roll, uint8_t* pu8_rxBuffCompass, uint8_t* pu8_rxBuffAccelSize) 
{
    uint8_t dataLsbX,
			dataMsbX,
			dataLsbY,
			dataMsbY,
            dataLsbZ,
            dataMsbZ;
	static int16_t dataX = 0,
                dataY = 0,
                dataZ = 0,
		        angle = 0;
	float xh,
		yh,
		rPitch,
		rRoll;
			
    dataLsbX = pu8_rxBuffCompass[1];
    dataMsbX = pu8_rxBuffCompass[0];
    dataLsbY = pu8_rxBuffCompass[5];
    dataMsbY = pu8_rxBuffCompass[4];
    dataLsbZ = pu8_rxBuffCompass[3];
    dataMsbZ = pu8_rxBuffCompass[2];

    dataX = (int16_t)((dataMsbX<<8) | dataLsbX);
    dataY = (int16_t)((dataMsbY<<8) | dataLsbY);
    dataZ = (int16_t)((dataMsbZ<<8) | dataLsbZ);
	
	rPitch = (M_PI/180)*d_pitch;
	rRoll = (M_PI/180)*d_roll;
  
	xh = ((float)(dataX - OFFSET_X) * cos(rPitch)) + ((float)(dataY - OFFSET_Y) * sin(rRoll) * sin(rPitch)) - ((float)(dataZ - OFFSET_Z) * cos(rRoll) * sin(rPitch));
	yh = ((float)(dataY - OFFSET_Y) * cos(rRoll)) + ((float)(dataZ - OFFSET_Z) * sin(rRoll));
  
	angle = (180/M_PI) * (atan2(-yh,xh)+ DECLINATION + OFFSET);
  
	return (int16_t)(-angle + 360) % 360;
}

static void _RUN_Mower_GetAnglePitchRoll(double* pd_pitch, double* pd_roll, uint8_t* pu8_rxBuffAccel, uint8_t* pu8_rxBuffAccelSize)
{
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

    dataLsbX = pu8_rxBuffAccel[0];
    dataMsbX = pu8_rxBuffAccel[1];
    dataLsbY = pu8_rxBuffAccel[2];
    dataMsbY = pu8_rxBuffAccel[3];
    dataLsbZ = pu8_rxBuffAccel[4];
    dataMsbZ = pu8_rxBuffAccel[5];
    
    dataX = ((float)(int16_t)((dataMsbX<<8) | dataLsbX))/256;
    dataY = ((float)(int16_t)((dataMsbY<<8) | dataLsbY))/256;
    dataZ = ((float)(int16_t)((dataMsbZ<<8) | dataLsbZ))/256;
	
    valuePitch = 180 * atan2(-dataX, sqrt(dataY*dataY + dataZ*dataZ))/M_PI;
    valueRoll = 180 * atan2(dataY, sqrt(dataX*dataX + dataZ*dataZ))/M_PI;
	
	*pd_pitch = (double)RUN_FIFO_GetPitchAverage((int16_t)valuePitch);
	*pd_roll = (double)RUN_FIFO_GetRollAverage((int16_t)valueRoll);
}

void RUN_Mower_TiltProtection(void)
{	
	if((gd_pitch <= PITCH_MIN) || (gd_pitch >= PITCH_MAX) || (gd_roll <= ROLL_MIN) || (gd_roll >= ROLL_MAX)) 
	{ 
		HAL_GPIO_UpdateBladeState(OFF);
	}
	else 
	{
		HAL_GPIO_UpdateBladeState(ON);
	}
}

void RUN_Mower_SonarDistance()
{
	//HAL_Sonar_Distance(); get from Sonar throught I2C - MVE
}

uint8_t RUN_Mower_WireDetection()
{
	static uint8_t _u8_wireState = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint8_t _u8_wireValue = 0;
	static uint16_t _u16_cptValue = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_wireState)
   	{
		default:
	  	case 0:
			_u16_randAngle = _RUN_Mower_MyRandDeg(360);
			_u16_startAngle = gu16_currentAngle;
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%360;
			
			RUN_PWM_Backward(MIDDLE_SPEED);

			if ( (gu16_distanceWireLeft > WIRE_DETECTION_LIMITE))
			{
				_u8_wireValue = 1;
			}
			else if (gu16_distanceWireRight > WIRE_DETECTION_LIMITE)
			{
				_u8_wireValue = 2;
			}
			_u8_wireState = 1;
			
			break;
		case 1:
			if ( (_u16_cptValue) >= GPT_ONE_SECOND )
			{
				if (_u8_wireValue == 1)
				{
					RUN_PWM_Stop();
					RUN_PWM_Right();
					_u16_cptValue = 0;
					_u8_wireState = 2;
				}
				else if (_u8_wireValue == 2)
				{
					RUN_PWM_Stop();
					RUN_PWM_Left();
					_u16_cptValue = 0;
					_u8_wireState = 2;
				}
				else
				{
					_u16_cptValue = 0;
					_u8_wireState = 2;
				}
			}
			else
			{
				_u16_cptValue++;
			}
			
			break;
		case 2 :
			if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%360)) && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%360)) )
			{
				_u8_wireState = 3;
			}
			else
			{
				gu16_distanceWireRight = HAL_ADC_GetRightWireValue();
				gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();

				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_wireState = 0;
					u8_returnValue = 2;
				}
				else if ((gu16_distanceWireLeft > WIRE_DETECTION_LIMITE) || (gu16_distanceWireRight > WIRE_DETECTION_LIMITE) )
				{
					_u8_wireState = 0;
				}
			}

			break;
		case 3 :
			RUN_PWM_Stop();
			_u8_wireState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

uint8_t RUN_Mower_WireDetectionOnReturn()
{
	static uint8_t _u8_wireState = 0;
	static uint8_t _u8_wireValue = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_wireState)
   	{
		default:
	  	case 0:			
			if ( (gu16_distanceWireLeft > WIRE_DETECTION_LIMITE))
			{
				_u8_wireValue = 1;
			}
			else if (gu16_distanceWireRight > WIRE_DETECTION_LIMITE)
			{
				_u8_wireValue = 2;
			}
			_u8_wireState = 1;
			
			break;
		case 1:
			if (_u8_wireValue == 1)
			{
				RUN_PWM_Stop();
				_u8_wireState = 0;
				u8_returnValue = 1;
			}
			else if (_u8_wireValue == 2)
			{
				RUN_PWM_Stop();
				RUN_PWM_Left();
				_u8_wireState = 2;
			}
			else
			{
				_u8_wireState = 0;
				u8_returnValue = 1;
			}
			
			break;
		case 2 :
			gu16_distanceWireRight = HAL_ADC_GetRightWireValue();
			gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();

			u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
			u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
			u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

			if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1) )
			{
				RUN_PWM_Stop();
				_u8_wireState = 0;
				u8_returnValue = 2;
			}
			else if ( gu16_distanceWireLeft > WIRE_DETECTION_LIMITE )
			{
				_u8_wireState = 3;
			}
			else if (gu16_distanceWireRight > WIRE_DETECTION_LIMITE)
			{
				RUN_PWM_Stop();
				_u8_wireState = 0;
			}
			
			break;
		case 3 :
			RUN_PWM_Stop();
			_u8_wireState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

uint8_t RUN_Mower_BumperDetection()
{
	static uint8_t _u8_bumperState = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint16_t _u16_cptValue = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_bumperState)
   	{
		default:
	  	case 0:
			_u16_randAngle = _RUN_Mower_MyRandDeg(360);
			_u16_startAngle = gu16_currentAngle;
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%360;

			RUN_PWM_Backward(MIDDLE_SPEED);

			_u8_bumperState = 1;
			
			break;
		case 1 :
			if ( (_u16_cptValue) >= GPT_ONE_SECOND )
			{
				RUN_PWM_Stop();
				RUN_PWM_Right();
				_u16_cptValue = 0;
				_u8_bumperState = 2;
			}
			else
			{
				_u16_cptValue++;
			}
			
			break;
		case 2 :
			if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%360)) && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%360)) )
			{
				_u8_bumperState = 3;
			}
			else
			{
				gu16_distanceWireRight = HAL_ADC_GetRightWireValue();
				gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();

				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_bumperState = 0;
				}
				else if ((gu16_distanceWireLeft > WIRE_DETECTION_LIMITE) || (gu16_distanceWireRight > WIRE_DETECTION_LIMITE) )
				{
					_u8_bumperState = 3;
				}
			}

			break;
		case 3 :
			RUN_PWM_Stop();
			_u8_bumperState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

uint8_t RUN_Mower_DirectionFromBase() 
{
	static uint16_t _u16_angleFromBase = 0;
	static uint8_t _u8_baseState = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_retunValue = 0;
	
	switch (_u8_baseState)
	{
		case 0:
			RUN_Mower_GetAzimut();
			_u16_angleFromBase = gu16_azimut;
			RUN_PWM_Right();

			_u8_baseState = 1;
			break;
		case 1 :
			if ( (gu16_currentAngle > ((_u16_angleFromBase - gu8_deltaAngle)%180)) && (gu16_currentAngle < ((_u16_angleFromBase + gu8_deltaAngle)%180)) )
			{
				_u8_baseState = 2;
			}
			else
			{
				gu16_distanceWireRight = HAL_ADC_GetRightWireValue();
				gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();

				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_baseState = 0;
					u8_retunValue = 2;
				}
				else if ((gu16_distanceWireLeft > WIRE_DETECTION_LIMITE) || (gu16_distanceWireRight > WIRE_DETECTION_LIMITE) )
				{
					RUN_PWM_Stop();
					_u8_baseState = 0;
					u8_retunValue = 3;
				}
			}

			break;
		case 2:
			_u8_baseState = 0;
			u8_retunValue = 1;
		default:
			break;
	}

	return u8_retunValue;
}

uint8_t RUN_Mower_RunMower()
{
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	//HAL_Sonar_GetDistance(&gu8_distanceFC, &gu8_distanceFL, &gu8_distanceFR); Get from I2C - MVE
	RUN_FIFO_GetSonarAverage(&gu8_distanceFC, &gu8_distanceFL, &gu8_distanceFR);

	gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();
	gu16_distanceWireRight = HAL_ADC_GetRightWireValue();

	u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
	u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
	u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

	if ( (gu16_distanceWireLeft > WIRE_DETECTION_LIMITE) || (gu16_distanceWireRight > WIRE_DETECTION_LIMITE) ) 
	{
		u8_returnValue = 1;
	}
	else if ((u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1)) 
	{
		u8_returnValue = 2;
	}
	else if ((gu8_distanceFC < SONAR_WARN) || (gu8_distanceFL < SONAR_WARN) || (gu8_distanceFR < SONAR_WARN))
	{
		RUN_PWM_Forward(MIDDLE_SPEED, MIDDLE_SPEED);
	}
	else 
	{
		RUN_PWM_Forward(HIGH_SPEED, HIGH_SPEED);
	}

	return u8_returnValue;
}

uint8_t RUN_Mower_WireGuiding()
{
	uint8_t u8_returnValue = 0;
	float f_Kp = 0.2;
			
	uint16_t u16_errorPosition = 0;
	float f_wirePwm = 0;
	
	gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();

	u16_errorPosition = WIRE_DETECTION_LOAD - gu16_distanceWireLeft;
	f_wirePwm = (f_Kp*(float)u16_errorPosition);
	
	if(f_wirePwm > MIDDLE_SPEED) {
		f_wirePwm = MIDDLE_SPEED;
	}
	else if(f_wirePwm < -MIDDLE_SPEED) {
		f_wirePwm = -MIDDLE_SPEED;
	}
	
	if(f_wirePwm > 0) 
	{
		RUN_PWM_Forward(MIDDLE_SPEED - f_wirePwm, MIDDLE_SPEED);
	}
	else if(f_wirePwm < 0) 
	{
		RUN_PWM_Forward(MIDDLE_SPEED, MIDDLE_SPEED - f_wirePwm);
	}
	else 
	{
		RUN_PWM_Forward(MIDDLE_SPEED, MIDDLE_SPEED);
	}	

	if ( (gu16_distanceWireLeft > WIRE_DETECTION_UNLOAD) && (gu16_distanceWireLeft < WIRE_DETECTION_LOAD) )
	{
		u8_returnValue = 1;
	}

	return u8_returnValue;	
}

uint16_t RUN_Mower_GetCurrentAngle()
{
	return gu16_currentAngle;
}

void RUN_Mower_SetEtatMower(EtatMower _eEtatMower)
{
	geEtatMower = _eEtatMower;
}

void RUN_Mower_SetErrorMower(ErrorMower _eErrorMower)
{
	geErrorMower = _eErrorMower;
}

EtatMower RUN_Mower_GetEtatMower(void)
{
	return geEtatMower;
}

ErrorMower RUN_Mower_GetErrorMower(void)
{
	return geErrorMower;
}
