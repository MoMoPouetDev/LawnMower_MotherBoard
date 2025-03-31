/*
 * RUN_BLE.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_BLE_RUN_BLE_H_
#define RUN_RUN_BLE_RUN_BLE_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>

/*--------------------------------------------------------------------------*/
/*! ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
typedef enum {
    UNKNOWN_COMMAND = 0x30,
    START_COMMAND,
    STOP_COMMAND,
    FORCE_START_COMMAND,
	DOCK_ON_COMMAND,
	DOCK_OFF_COMMAND,
    RAIN_ON_COMMAND,
	RAIN_OFF_COMMAND
}CommandMower;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_BLE_Init(void);
void RUN_BLE_SendStatus(void);
void RUN_BLE_ReceiveStatus(void);

#endif /* RUN_RUN_BLE_RUN_BLE_H_ */
