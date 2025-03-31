/*
 * HAL_Sonar.c
 *
 *  Created on: 26 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HAL_Timer.h"
#include "HAL_GPIO.h"
#include "HAL_Sonar.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
static uint8_t gu8_distanceSonarFC;
static uint8_t gu8_distanceSonarFL;
static uint8_t gu8_distanceSonarFR;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
uint8_t HAL_Sonar_GetEchoState(void);
uint8_t HAL_Sonar_DistanceCalculation(void);
void HAL_Sonar_SendPulse(uint8_t sonarID);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_Sonar_Init() 
{
    gu8_distanceSonarFC = 255;
    gu8_distanceSonarFL = 255;
    gu8_distanceSonarFR = 255;
}

void HAL_Sonar_Distance()
{
    static uint8_t u8_sonarState = 0;
    uint8_t u8_echoState = 0;

    switch (u8_sonarState)
    {
        case 0:
            HAL_Sonar_SendPulse(E_CENTER_TRIGGER_SONAR);
            u8_sonarState = 1;

            break;
        case 1:
            u8_echoState = HAL_Sonar_GetEchoState();
            if (u8_echoState == 2)
            {
                gu8_distanceSonarFC = HAL_Sonar_DistanceCalculation();
                u8_sonarState = 2;
            }

            break;
        case 2:
            HAL_Sonar_SendPulse(E_LEFT_TRIGGER_SONAR);
            u8_sonarState = 0;

            break;
        case 3:
            u8_echoState = HAL_Sonar_GetEchoState();
            if (u8_echoState == 2)
            {
                gu8_distanceSonarFL = HAL_Sonar_DistanceCalculation();
                u8_sonarState = 4;
            }

            break;
        case 4:
            HAL_Sonar_SendPulse(E_RIGHT_TRIGGER_SONAR);
            u8_sonarState = 5;

            break;
        case 5:
            u8_echoState = HAL_Sonar_GetEchoState();
            if (u8_echoState == 2)
            {
                gu8_distanceSonarFR = HAL_Sonar_DistanceCalculation();
                u8_sonarState = 0;
            }
            
            break;
        default:
            u8_sonarState = 0;
            break;
    }
}

void HAL_Sonar_SendPulse(uint8_t sonarID)
{       
    HAL_GPIO_WritePinSonar(sonarID, 1);
    HAL_TIMER_StartSonarPulse();
    HAL_GPIO_WritePinSonar(sonarID, 0);
}

uint8_t HAL_Sonar_GetEchoState( )
{
    uint8_t u8_returnValue = 0;

    u8_returnValue = HAL_GPIO_GetEchoState();

    return u8_returnValue;
}

uint8_t HAL_Sonar_DistanceCalculation()
{
    uint32_t distance = SONAR_DIST_ERR;
    uint32_t tempCount = 0;
    uint32_t u32_timerValue;

    u32_timerValue = HAL_GPIO_GetTimerValue();

    tempCount = u32_timerValue;
    
    distance = (tempCount / TIMER_DISTANCE)/2;
    
    if(distance > THRESHOLD_8_BITS) {
	    distance = THRESHOLD_8_BITS;
	}
	
    return (uint8_t)distance;
}

void HAL_Sonar_GetDistance(uint8_t* u8_distFC, uint8_t* u8_distFL, uint8_t* u8_distFR)
{
    *u8_distFC = gu8_distanceSonarFC;
    *u8_distFL = gu8_distanceSonarFL;
    *u8_distFR = gu8_distanceSonarFR;
}
