/*
 * FSM_Enroll.h
 *
 *  Created on: 14 JUL 2026
 *      Author: morgan.venandy
 */

#ifndef FSM_FSM_ENROLL_FSM_ENROLL_H_
#define FSM_FSM_ENROLL_FSM_ENROLL_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "FSM_Enum.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Enroll_Init();
void FSM_Enroll(S_MOWER_FSM_STATE e_FSM_Init_State);

#endif /* FSM_FSM_ENROLL_FSM_ENROLL_H_ */
