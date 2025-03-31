/*
 * RUN_Task_Interface.c
 *
 *  Created on: 5 oct. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
static uint32_t gArraySlotTask[NB_SLOT_CYCLIC_TASK] =
{
      CYCLIC_TASK_ADC_READ_VALUE , CYCLIC_TASK_READ_START_BUTTON, CYCLIC_TASK_READ_STOP_BUTTON,// 0
	  CYCLIC_TASK_GPS_ACQUISITION,  // 1
	  CYCLIC_TASK_ANGLE_READ, CYCLIC_TASK_SENSOR_READ, CYCLIC_TASK_UPDATE_LED, // 2
	  CYCLIC_TASK_SONAR, CYCLIC_TASK_TILT_PROTECTION, // 3
	  CYCLIC_TASK_WIRE_DETECTION, CYCLIC_TASK_BUMPER_DETECTION, // 4
	  CYCLIC_TASK_UPDATE_LED,  // 5
	  CYCLIC_TASK_BLE_SEND_STATUS,  // 6
	  CYCLIC_TASK_LEAVE_DOCK,  // 7
	  CYCLIC_TASK_RUN_MOWER,  // 8
	  CYCLIC_TASK_WIRE_GUIDING,  // 9
};

/*--------------------------------------------------------------------------*/
/* ... DATAS ...                                                            */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* ... LOCAL FUNCTIONS DECLARATIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* ... FUNCTIONS IMPLEMENTATIONS...                                         */
/*--------------------------------------------------------------------------*/
void RUN_Task_Interface_Init()
{
	RUN_Task_Sequencer_Init();
	RUN_Task_SetArraySlotTask(gArraySlotTask, NB_SLOT_CYCLIC_TASK);
}

