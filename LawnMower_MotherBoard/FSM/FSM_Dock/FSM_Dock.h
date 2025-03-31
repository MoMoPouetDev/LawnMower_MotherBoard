/*
 * FSM_Dock.h
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef FSM_FSM_DOCK_FSM_DOCK_H_
#define FSM_FSM_DOCK_FSM_DOCK_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "FSM_Enum.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Dock_Init();
void FSM_Dock(S_MOWER_FSM_STATE e_FSM_Operative_State);

#endif /* FSM_FSM_DOCK_FSM_DOCK_H_ */
