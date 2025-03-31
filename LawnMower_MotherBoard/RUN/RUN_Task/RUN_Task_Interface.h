/*
 * RUN_Task_Interface.h
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_TASK_RUN_TASK_INTERFACE_H_
#define RUN_RUN_TASK_RUN_TASK_INTERFACE_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define NB_SLOT_CYCLIC_TASK 20u

#define CYCLIC_TASK_ADC_READ_VALUE	    0x00000001
#define CYCLIC_TASK_LEAVE_DOCK  	    0x00000002
#define CYCLIC_TASK_READ_START_BUTTON  	0x00000004
#define CYCLIC_TASK_READ_STOP_BUTTON  	0x00000008
#define CYCLIC_TASK_RUN_MOWER         	0x00000010
#define CYCLIC_TASK_ANGLE_READ			0x00000020
#define CYCLIC_TASK_SONAR				0x00000040
#define CYCLIC_TASK_SENSOR_READ			0x00000080
#define CYCLIC_TASK_TILT_PROTECTION		0x00000100
#define CYCLIC_TASK_WIRE_DETECTION		0x00000200
#define CYCLIC_TASK_BUMPER_DETECTION	0x00000400
#define CYCLIC_TASK_GPS_ACQUISITION		0x00000800
#define CYCLIC_TASK_UPDATE_LED			0x00001000
#define CYCLIC_TASK_BLE_SEND_STATUS		0x00002000
#define CYCLIC_TASK_WIRE_GUIDING        0x00004000

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_Task_Interface_Init(void);

#endif /* RUN_RUN_TASK_RUN_TASK_INTERFACE_H_ */
