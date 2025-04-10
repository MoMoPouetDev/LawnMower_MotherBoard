/*
 * RUN_FIFO.c
 *
 *  Created on: 20 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "RUN_FIFO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define FIFO_SIZE 10

Fifo fifoLeftWire;
Fifo fifoRightWire;
Fifo fifoPitch;
Fifo fifoRoll;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _RUN_FIFO_InitList(Fifo *, int);
static void _RUN_FIFO_AddElement(Fifo *, int);
static void _RUN_FIFO_RemoveElement(Fifo *);
static int16_t _RUN_FIFO_GetAverage(Fifo *list, int value);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_FIFO_Init()
{
	_RUN_FIFO_InitList(&fifoLeftWire, 0);
	_RUN_FIFO_InitList(&fifoRightWire, 0);
	_RUN_FIFO_InitList(&fifoPitch, 0);
	_RUN_FIFO_InitList(&fifoRoll, 0);
}

static void _RUN_FIFO_InitList(Fifo *list, int value) {
	Element *element = malloc(sizeof(*element));
	if ((list == NULL) || (element == NULL)) {
		exit(1);
	}
	
	element->value = value;
	element->nextElement = NULL;
	
	list->firstElement = element;
	
	for (int i = 0; i < FIFO_SIZE - 1; i++)
	{
		_RUN_FIFO_AddElement(list, value);
	}
}

static void _RUN_FIFO_AddElement(Fifo *list, int value) {
	Element *newElement = malloc(sizeof(*newElement));
	if (newElement == NULL)
	{
		exit(1);
	}
	
	Element *actual = list->firstElement;
	
	while(actual->nextElement != NULL) {
		actual = actual->nextElement;
	}
	
	actual->nextElement = newElement;
	
	newElement->value = value;
	newElement->nextElement = NULL;
}

static void _RUN_FIFO_RemoveElement(Fifo *list) {
	if (list == NULL)
	{
		exit(1);
	}
	
	Element *removedElement = list->firstElement;
	list->firstElement = list->firstElement->nextElement;
	free(removedElement);
}

static int16_t _RUN_FIFO_GetAverage(Fifo *list, int value) {
	int16_t average = 0;
	int count = 0;
	
	_RUN_FIFO_AddElement(list, value);	
	_RUN_FIFO_RemoveElement(list);
	
	Element *actual = list->firstElement;
	while (actual != NULL)
	{
		count++;
		average = average + actual->value;
		actual = actual->nextElement;
	}
	
	average = average / count;
	
	return average;
}

int16_t RUN_FIFO_GetPitchAverage(int16_t s16_value)
{
	int16_t s16_returnValue;

	s16_returnValue = _RUN_FIFO_GetAverage(&fifoPitch, s16_value);
	
	return s16_returnValue;
}

int16_t RUN_FIFO_GetRollAverage(int16_t s16_value)
{
	int16_t s16_returnValue;

	s16_returnValue = _RUN_FIFO_GetAverage(&fifoRoll, s16_value);
	
	return s16_returnValue;
}
