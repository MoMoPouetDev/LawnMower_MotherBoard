/*
 * FSM_Error.h
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef FSM_FSM_ERROR_FSM_ERROR_H_
#define FSM_FSM_ERROR_FSM_ERROR_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "FSM_Enum.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Error_Init();
void FSM_Error(S_MOWER_FSM_STATE e_FSM_Error_State);

#endif /* FSM_FSM_ERROR_FSM_ERROR_H_ */
