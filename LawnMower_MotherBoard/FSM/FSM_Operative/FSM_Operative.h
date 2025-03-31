/*
 * FSM_Operative.h
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

#ifndef FSM_FSM_OPERATIVE_FSM_OPERATIVE_H_
#define FSM_FSM_OPERATIVE_FSM_OPERATIVE_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "FSM_Enum.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Operative_Init();
void FSM_Operative(S_MOWER_FSM_STATE e_FSM_Operative_State);

#endif /* FSM_FSM_OPERATIVE_FSM_OPERATIVE_H_ */
