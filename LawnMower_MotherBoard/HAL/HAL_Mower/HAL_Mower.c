/*
 * HAL_Mower.c
 *
 *  Created on: 05 MAR 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "HAL_FIFO.h"
#include "HAL_GPIO.h"
#include "HAL_Mower.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define M_PI 3.14
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

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_Mower_Init() 
{

}

uint16_t HAL_Mower_MyRandDeg(uint16_t u16_modulo)
{
    return (uint16_t)((rand()%u16_modulo) + 1);
}

int16_t HAL_Mower_GetAngleFromNorth(double d_pitch, double d_roll, uint8_t* pu8_rxBuffCompass, uint8_t* pu8_rxBuffAccelSize) 
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

void HAL_Mower_GetAnglePitchRoll(double* pd_pitch, double* pd_roll, uint8_t* pu8_rxBuffAccel, uint8_t* pu8_rxBuffAccelSize)
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
	
	*pd_pitch = (double)HAL_FIFO_GetPitchAverage((int16_t)valuePitch);
	*pd_roll = (double)HAL_FIFO_GetRollAverage((int16_t)valueRoll);
}

void HAL_Mower_TiltProtection(double d_pitch, double d_roll)
{	
	if((d_pitch <= PITCH_MIN) || (d_pitch >= PITCH_MAX) || (d_roll <= ROLL_MIN) || (d_roll >= ROLL_MAX)) { 
		HAL_GPIO_UpdateBladeState(OFF);
	}
	else {
		HAL_GPIO_UpdateBladeState(ON);
	}
}
