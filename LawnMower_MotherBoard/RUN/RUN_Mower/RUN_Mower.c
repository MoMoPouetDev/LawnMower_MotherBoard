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
#include <stdlib.h>
#include <math.h>
#include "HAL_Timer.h"
#include "HAL_GPIO.h"
#include "RUN_PWM.h"
#include "RUN_FIFO.h"
#include "RUN_Mower.h"
#include "RUN_Sensors.h"
#include "RUN_I2C.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define SENSORS_TIMER_ONE_SECOND	1000
/*** Motor ***/
#define HIGH_SPEED 100
#define MIDDLE_SPEED 90
#define LOW_SPEED 80
/*** Timer ***/
#define GPT_ONE_SECOND 100
#define GPT_FIVE_SECOND 500
#define GPT_SEVEN_SECOND 700
/*** GPS ***/
#define COORDINATES_BASE_LAT 49.2315928
#define COORDINATES_BASE_LONG 1.2470619
#define DEG_TO_RAD(d) ((d) * M_PI / 180.0)
#define COORDINATES_BASE_LAT_RAD  DEG_TO_RAD(COORDINATES_BASE_LAT)
#define COORDINATES_BASE_LONG_RAD DEG_TO_RAD(COORDINATES_BASE_LONG)
/*** Time to Mow ***/
#define THRESHOLD_HOUR_MIN 9
#define THRESHOLD_HOUR_MAX 18
/*** Compass ***/
#define DELTA_ANGLE 20
#define HEADING_KP          0.30f
#define HEADING_KD          0.8f
#define HEADING_CORR_MAX    15.0f
//#define M_PI 3.14
#define DECLINATION ((54)*(M_PI/(60*180))) //0.015
#define OFFSET -100
#define CALIBRATION_X_MAX 406
#define CALIBRATION_X_MIN 158
#define CALIBRATION_Y_MAX 424
#define CALIBRATION_Y_MIN 190
#define CALIBRATION_Z_MAX -8
#define CALIBRATION_Z_MIN -198
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
#define SONAR_LIMITE 15
#define SONAR_ERR 5
#define SONAR_DIST_ERR 999
/*** Variables ***/
static uint8_t gu8_deltaAngle;
static uint8_t gu8_timeToMow;
static uint8_t gu8_enrolled;
static uint16_t gu16_currentAngle;
static uint16_t gu16_azimut;
static int16_t gs16_pitch;
static int8_t gs8_roll;
static EtatMower geEtatMower;
static ErrorMower geErrorMower;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static uint16_t _RUN_Mower_MyRandDeg(uint16_t u16_modulo);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Mower_Init(void)
{
	gu8_deltaAngle = DELTA_ANGLE;
	gu8_timeToMow = 0;
	gu8_enrolled = 0;
	gs16_pitch = 0;
	gs8_roll = 0;
	gu16_currentAngle = 0;
	gu16_azimut = 0;
}

uint8_t RUN_Mower_IsTimeToMow(void)
{
	return 1;//gu8_timeToMow;
}

void RUN_Mower_SetTimeToMow(uint8_t u8_timeToMow)
{
	gu8_timeToMow = u8_timeToMow;
}

uint8_t RUN_Mower_IsEnrolled(void)
{
	return gu8_enrolled;
}

void RUN_Mower_SetEnrolled(uint8_t u8_enrolled)
{
	gu8_enrolled = u8_enrolled;
}

uint8_t RUN_Mower_LeaveDockCharger(void)
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

void RUN_Mower_GetAngles(void)
{
	static uint8_t _u8_getAngleState = 0;
	static uint16_t _u16_sensorsCpt = 0;
	E_I2C_USED e_i2cUsed = E_I2C_USED_NONE;

	e_i2cUsed = RUN_I2C_GetUsed();
	
	if (_u16_sensorsCpt >= 2)
	{
		if ((e_i2cUsed == E_I2C_USED_NONE) || (e_i2cUsed == E_I2C_USED_ANGLES))
		{
			RUN_I2C_SetUsed(E_I2C_USED_ANGLES);
			switch (_u8_getAngleState)
			{
				case 0 :
					_u8_getAngleState++;				
					break;

				case 1 :
					HAL_I2C_ReadAccel(&gs16_pitch, &gs8_roll);
					_u8_getAngleState++;
					RUN_I2C_SetUsed(E_I2C_USED_NONE);
					break;

				case 2 :
					gu16_currentAngle = HAL_I2C_ReadCompass();
					_u8_getAngleState++;
					RUN_I2C_SetUsed(E_I2C_USED_NONE);
					break;

				default:
					_u8_getAngleState = 0;
					_u16_sensorsCpt = 0;
					RUN_I2C_SetUsed(E_I2C_USED_NONE);
					break;
			}
		}
	}
	else
	{
		_u16_sensorsCpt++;
	}
}

void RUN_Mower_GetAzimut(void)
{
	float f_latitude;
	float f_longitude;
	float f_dLon;
	float x = 0.0;
	float y = 0.0;
	float f_azimutRad;
    float f_azimutDeg;

	f_latitude = RUN_Sensors_GetLatitude()  * (float)(M_PI / 180.0);
	f_longitude = RUN_Sensors_GetLongitude()  * (float)(M_PI / 180.0);
	
	x = cos(f_latitude)*sin(COORDINATES_BASE_LAT_RAD) - sin(f_latitude)*cos(COORDINATES_BASE_LAT_RAD)*cos(COORDINATES_BASE_LONG_RAD-f_longitude);
	y = sin(COORDINATES_BASE_LONG_RAD-f_longitude)*cos(COORDINATES_BASE_LAT_RAD);

	f_azimutRad = atan2(y, x);
    f_azimutDeg = f_azimutRad * (float)(180.0 / M_PI);
    if (f_azimutDeg < 0.0f) 
	{ 
		f_azimutDeg += 360.0f; 
	}

    gu16_azimut = (uint16_t)f_azimutDeg;
}

void RUN_Mower_TiltProtection(void)
{	
	if((gs16_pitch <= PITCH_MIN) || (gs16_pitch >= PITCH_MAX) || (gs8_roll <= ROLL_MIN) || (gs8_roll >= ROLL_MAX)) 
	{ 
		/** BRAKE **/
		HAL_GPIO_RequestBladeBrake();
	}
	else 
	{
		HAL_GPIO_RequestBladeRelease();
	}
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
	uint8_t u8_distanceSonarFC     = 0;
    uint8_t u8_distanceSonarFL     = 0;
    uint8_t u8_distanceSonarFR     = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_bumperState)
   	{
		default:
	  	case 0:
			_u16_randAngle = _RUN_Mower_MyRandDeg(360);
			_u16_startAngle = gu16_currentAngle;
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%360;

			RUN_PWM_Backward(MIDDLE_SPEED);
			_u16_cptValue = 0;
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
				_u16_cptValue = 0;
			}
			else
			{
				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				u8_distanceSonarFC = RUN_Sensors_GetDistanceSonarFC();
                u8_distanceSonarFL = RUN_Sensors_GetDistanceSonarFL();
                u8_distanceSonarFR = RUN_Sensors_GetDistanceSonarFR();

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_bumperState = 0;
				}
				else if ((u8_distanceSonarFC <= SONAR_LIMITE)
                      || (u8_distanceSonarFL <= SONAR_LIMITE)
                      || (u8_distanceSonarFR <= SONAR_LIMITE))
                {
                    RUN_PWM_Stop();
                    _u8_bumperState = 3;
                }
				else if (_u16_cptValue > GPT_SEVEN_SECOND)
				{
					_u8_bumperState = 3;
					_u16_cptValue = 0;
				}
				_u16_cptValue++;
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

uint8_t RUN_Mower_SonarDetection(void)
{
    static uint8_t  _u8_sonarState   = 0;
    static uint16_t _u16_randAngle   = 0;
    static uint16_t _u16_startAngle  = 0;
    static uint16_t _u16_endAngle    = 0;
    static uint16_t _u16_cptValue    = 0;
	uint8_t u8_leftBumperState       = 0;
    uint8_t u8_centerBumperState     = 0;
    uint8_t u8_rightBumperState      = 0;
    uint8_t u8_distanceSonarFC       = 0;
    uint8_t u8_distanceSonarFL       = 0;
    uint8_t u8_distanceSonarFR       = 0;
    uint8_t u8_returnValue           = 0;

    switch(_u8_sonarState)
    {
        default:
        case 0:
            _u16_randAngle  = _RUN_Mower_MyRandDeg(360);
            _u16_startAngle = gu16_currentAngle;
            _u16_endAngle   = (_u16_startAngle + _u16_randAngle) % 360;

            RUN_PWM_Backward(MIDDLE_SPEED);

            _u8_sonarState = 1;

            break;

        case 1:
            if (_u16_cptValue >= GPT_ONE_SECOND)
            {
                RUN_PWM_Stop();
                RUN_PWM_Right();
                _u16_cptValue  = 0;
                _u8_sonarState = 2;
            }
            else
            {
                _u16_cptValue++;
            }

            break;

        case 2:
            if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle) % 360))
              && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle) % 360)) )
            {
                _u8_sonarState = 3;
            }
            else
            {
				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

                u8_distanceSonarFC = RUN_Sensors_GetDistanceSonarFC();
                u8_distanceSonarFL = RUN_Sensors_GetDistanceSonarFL();
                u8_distanceSonarFR = RUN_Sensors_GetDistanceSonarFR();

				if ((u8_distanceSonarFC <= SONAR_LIMITE)
                      || (u8_distanceSonarFL <= SONAR_LIMITE)
                      || (u8_distanceSonarFR <= SONAR_LIMITE))
                {
                    RUN_PWM_Stop();
                    _u8_sonarState = 0;
                }
				else if ((u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1))
				{
					RUN_PWM_Stop();
					_u8_sonarState = 3;
				}
            }

            break;

        case 3:
            RUN_PWM_Stop();
            _u8_sonarState = 0;
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
				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_baseState = 0;
					u8_retunValue = 2;
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
	/* Regulation */
	static uint16_t _u16_targetHeading = 0xFFFF;
    static int16_t  _s16_prevError = 0;
    int16_t s16_error = 0;
    int16_t s16_dError = 0;
    float   f_correction = 0.0;
    uint8_t u8_speedLeft = 0;
    uint8_t u8_speedRight = 0;
	/*** ***/
	uint8_t u8_distanceSonarFC = 0;
	uint8_t u8_distanceSonarFL = 0;
	uint8_t u8_distanceSonarFR = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	u8_distanceSonarFC = RUN_Sensors_GetDistanceSonarFC();
	u8_distanceSonarFL = RUN_Sensors_GetDistanceSonarFL();
	u8_distanceSonarFR = RUN_Sensors_GetDistanceSonarFR();

	u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
	u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
	u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

	if ((u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_rightBumperState == 1)) 
	{
		_u16_targetHeading = 0xFFFF;
		_s16_prevError = 0;
		u8_returnValue = 2;
	}
	else if ((u8_distanceSonarFC <= SONAR_LIMITE)
      || (u8_distanceSonarFL <= SONAR_LIMITE)
      || (u8_distanceSonarFR <= SONAR_LIMITE))
	{
		_u16_targetHeading = 0xFFFF;
		_s16_prevError     = 0;
		u8_returnValue     = 3;
	}
	else 
	{
		/* --- Capture du cap cible au premier appel après un reset --- */
        if (_u16_targetHeading == 0xFFFF)
        {
            _u16_targetHeading = gu16_currentAngle;
            _s16_prevError = 0;
        }

        /* --- Calcul erreur avec wrap-around 0/360 --- */
        s16_error = (int16_t)_u16_targetHeading - (int16_t)gu16_currentAngle;
        if (s16_error >  180) { s16_error -= 360; }
        if (s16_error < -180) { s16_error += 360; }

        /* --- Terme dérivé --- */
        s16_dError = s16_error - _s16_prevError;
        _s16_prevError = s16_error;

        /* --- Correction PD --- */
        f_correction = (HEADING_KP * (float)s16_error) + (HEADING_KD * (float)s16_dError);
        if (f_correction >  HEADING_CORR_MAX) { f_correction =  HEADING_CORR_MAX; }
        if (f_correction < -HEADING_CORR_MAX) { f_correction = -HEADING_CORR_MAX; }

        /* --- Application sur les deux moteurs --- */
        if ((u8_distanceSonarFC < SONAR_WARN) || (u8_distanceSonarFL < SONAR_WARN) || (u8_distanceSonarFR < SONAR_WARN))
        {
            u8_speedLeft  = (uint8_t)((float)MIDDLE_SPEED + f_correction);
            u8_speedRight = (uint8_t)((float)MIDDLE_SPEED - f_correction);
        }
        else
        {
            u8_speedLeft  = (uint8_t)((float)HIGH_SPEED + f_correction);
            u8_speedRight = (uint8_t)((float)HIGH_SPEED - f_correction);
        }
        RUN_PWM_Forward(u8_speedLeft, u8_speedRight);
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
