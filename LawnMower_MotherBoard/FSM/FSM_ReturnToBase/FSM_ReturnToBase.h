/*
 * FSM_ReturnToBase.h
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef FSM_FSM_RETURNTOBASE_FSM_RETURNTOBASE_H_
#define FSM_FSM_RETURNTOBASE_FSM_RETURNTOBASE_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "FSM_Enum.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_ReturnToBase_Init();
void FSM_ReturnToBase(S_MOWER_FSM_STATE e_FSM_Error_State);

#endif /* FSM_FSM_RETURNTOBASE_FSM_RETURNTOBASE_H_ */
