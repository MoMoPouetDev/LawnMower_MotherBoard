/**
 * \file RUN_Task.c
 * \brief Cyclic activity handler
 *
 * \author morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "assert.h"

#include "RUN_Task.h"
/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/* ... DATAS ...                                                            */
/*--------------------------------------------------------------------------*/
/* Volatile rational : this variable is Read or Write during different "thread":
 * main or interrupt callback
 */
volatile static uint32_t gu32_TickCounter;
volatile static uint32_t gu32_Cyclic_Task_Event;
uint32_t *gpu32_ArraySlotTaskReference;
uint8_t gu8_ArraySlotTaskSize;
/*--------------------------------------------------------------------------*/
/* ... LOCAL FUNCTIONS DECLARATIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* ... FUNCTIONS IMPLEMENTATIONS...                                         */
/*--------------------------------------------------------------------------*/

/**
 * \fn void RUN_Tasks_Init(void)
 * \brief Initialize the variables used to run the OS tasks
*  \details Initialize the variables used to run the OS tasks
*
*/
void RUN_Task_Sequencer_Init(void)
{
	gu32_TickCounter = 0;
	gu32_Cyclic_Task_Event = 0;
	gu8_ArraySlotTaskSize = 0;
}

/**
 * \fn u32 RUN_Tasks_GetCyclicTask(void)
 * \brief Get the cyclic task flag
 * return g_u32_Cyclic_Task_Event u32
 */
uint32_t RUN_Task_GetCyclicTask(void)
{
	return gu32_Cyclic_Task_Event;
}

/**
 * \fn void RUN_Tasks_EraseCyclicTask(u32 u32_ErasingTask)
 * \brief Erase the desired cyclic task
 * \details Erase the desired cyclic task
 * \param[in] u32_ErasingTask u32
 */
void RUN_Task_EraseCyclicTask(uint32_t u32_ErasingTask)
{
	gu32_Cyclic_Task_Event &= ~u32_ErasingTask;
}

void RUN_Task_TickIncrement(void)
{
	assert(gpu32_ArraySlotTaskReference);

	uint8_t u8_Tick;

	gu32_TickCounter++;

	u8_Tick = gu32_TickCounter % gu8_ArraySlotTaskSize;

	gu32_Cyclic_Task_Event |= gpu32_ArraySlotTaskReference[u8_Tick];
}

void RUN_Task_SetArraySlotTask(uint32_t *tu32_ArraySlotTask, uint8_t u8_Size)
{
	assert(tu32_ArraySlotTask);
	assert(u8_Size > 0);

	gu8_ArraySlotTaskSize = u8_Size;
	gpu32_ArraySlotTaskReference = tu32_ArraySlotTask;

}

 /* eof RUN_Task.c */
