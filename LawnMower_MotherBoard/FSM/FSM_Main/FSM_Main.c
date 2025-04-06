/*
 * FSM_Main.c
 *
 *  Created on: 7 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Init.h"
#include "RUN_ADC.h"
#include "RUN_BLE.h"
#include "RUN_GPIO.h"
#include "RUN_Mower.h"
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"

#include "FSM_Enum.h"
#include "FSM_Init.h"
#include "FSM_Dock.h"
#include "FSM_Operative.h"
#include "FSM_ReturnToBase.h"
#include "FSM_Error.h"
#include "FSM_Main.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
S_MOWER_FSM_STATE ge_FSM_Phase;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _FSM_Main_GpsAcquisition(uint32_t u32_CyclicTask);
static void _FSM_Main_UpdateLed(uint32_t u32_CyclicTask);
static void _FSM_Main_GetAngles(uint32_t u32_CyclicTask);
static void _FSM_Main_ADCRead(uint32_t u32_CyclicTask);
static void _FSM_Main_SendStatus(uint32_t u32_CyclicTask);
static void _FSM_Main_TiltProtection(uint32_t u32_CyclicTask);
static void _FSM_Main_UpdateFsmMower(void);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
/**
 * \fn void FSM_Main_Init( void )
 * \brief This function initialize all finite state machine
 * \details - Initialize Event finite state machine
 * - by calling Events_Init()
 * - by calling FSM_Init_Init()
 * - by calling FSM_StopButton_Init()
**/
void FSM_Main_Init( void )
{
	/*** FSM Init ***/
	ge_FSM_Phase = FSM_Enum_GetFsmPhase();
	FSM_Init_Init();
	FSM_Dock_Init();
	FSM_Operative_Init();
	FSM_ReturnToBase_Init();
	FSM_Error_Init();

	/*** RUN Init ***/
	RUN_Init();
}

void FSM_Main( void )
{
   while(1)
   {
		uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
		u32_CyclicTask = RUN_Task_GetCyclicTask();
		
		_FSM_Main_UpdateLed(u32_CyclicTask);
		_FSM_Main_GetAngles(u32_CyclicTask);
		_FSM_Main_ADCRead(u32_CyclicTask);
		_FSM_Main_SendStatus(u32_CyclicTask);
		_FSM_Main_TiltProtection(u32_CyclicTask);
      /***************************************************************************************************************/
      /*                                  DEBUG                                                   */
      /***************************************************************************************************************/
		//FSM_TEST_SonarDistance(u32_CyclicTask);

	  /***************************************************************************************************************/
	  /*                                   FINITE STATE MACHINE                                                      */
	  /***************************************************************************************************************/

		if (ge_FSM_Phase == PHASE_INIT_INIT)
		{
			FSM_Init( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_DOCK_INIT) && (ge_FSM_Phase <= PHASE_DOCK_WAITING_FOR_LEAVING_DOCK))
		{
			FSM_Dock( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_OPERATIVE_INIT) && (ge_FSM_Phase <= PHASE_OPERATIVE_WAITING_FOR_RETURN_TO_BASE) )
		{
			FSM_Operative( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_RETURN_TO_BASE_INIT) && (ge_FSM_Phase <= PHASE_RETURN_TO_BASE_WAITING_DOCKING) )
		{
			FSM_ReturnToBase( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_ERROR_INIT) && (ge_FSM_Phase <= PHASE_ERROR_INIT) )
		{
			FSM_Error( ge_FSM_Phase );
		}
		FSM_Main_UpdateFsmMower();
   }
}

static void _FSM_Main_UpdateFsmMower()
{
	ge_FSM_Phase = FSM_Enum_GetFsmPhase();
}

static void _FSM_Main_GpsAcquisition(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_GPS_ACQUISITION) != 0) {
		// Data slave Acquisition - RUN Sensors - MVE
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_GPS_ACQUISITION);
	}
}

static void _FSM_Main_UpdateLed(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_UPDATE_LED) != 0) {
		// Update Leds on slave maybe on run mower - MVE
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_UPDATE_LED);
	}
}

static void _FSM_Main_GetAngles(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_ANGLE_READ) != 0) {
		RUN_Mower_GetAngles();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ANGLE_READ);
	}
}

static void _FSM_Main_ADCRead(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		RUN_ADC_ReadValue();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
}

static void _FSM_Main_SendStatus(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_BLE_SEND_STATUS) != 0) {
		RUN_BLE_SendStatus();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_BLE_SEND_STATUS);
	}
}

static void _FSM_Main_TiltProtection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_TILT_PROTECTION) != 0) {
		RUN_Mower_TiltProtection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_TILT_PROTECTION);
	}
}

static void _FSM_TEST(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR) != 0) {

		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR);
	}
}
