/*
 * RUN_Task.h
 *
 *  Created on: 11 avr. 2022
 *      Author: morgan.venandy
 */

#ifndef RUN_TASK_H_
#define RUN_TASK_H_

#include <stdint.h>

void RUN_Task_Sequencer_Init(void);
void RUN_Task_TickIncrement(void);
uint32_t RUN_Task_GetCyclicTask(void);
void RUN_Task_SetArraySlotTask(uint32_t *tu32_ArraySlotTask, uint8_t u8_Size);
void RUN_Task_EraseCyclicTask(uint32_t u32_ErasingTask);

#endif /* RUN_TASK_H_ */
