/*
 * HAL_FIFO.c
 *
 *  Created on: 20 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "HAL_FIFO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define FIFO_SIZE 10

Fifo fifoSonarFC;
Fifo fifoSonarFL;
Fifo fifoSonarFR;
Fifo fifoLeftWire;
Fifo fifoRightWire;
Fifo fifoPitch;
Fifo fifoRoll;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_FIFO_InitList(Fifo *, int);
void HAL_FIFO_AddElement(Fifo *, int);
void HAL_FIFO_RemoveElement(Fifo *);
int16_t HAL_FIFO_GetAverage(Fifo *list, int value);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_FIFO_Init()
{
	HAL_FIFO_InitList(&fifoSonarFC, 255);
	HAL_FIFO_InitList(&fifoSonarFL, 255);
	HAL_FIFO_InitList(&fifoSonarFR, 255);
	HAL_FIFO_InitList(&fifoLeftWire, 0);
	HAL_FIFO_InitList(&fifoRightWire, 0);
	HAL_FIFO_InitList(&fifoPitch, 0);
	HAL_FIFO_InitList(&fifoRoll, 0);
}

void HAL_FIFO_InitList(Fifo *list, int value) {
	Element *element = malloc(sizeof(*element));
	if ((list == NULL) || (element == NULL)) {
		exit(1);
	}
	
	element->value = value;
	element->nextElement = NULL;
	
	list->firstElement = element;
	
	for (int i = 0; i < FIFO_SIZE - 1; i++)
	{
		HAL_FIFO_AddElement(list, value);
	}
}

void HAL_FIFO_AddElement(Fifo *list, int value) {
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

void HAL_FIFO_RemoveElement(Fifo *list) {
	if (list == NULL)
	{
		exit(1);
	}
	
	Element *removedElement = list->firstElement;
	list->firstElement = list->firstElement->nextElement;
	free(removedElement);
}

int16_t HAL_FIFO_GetAverage(Fifo *list, int value) {
	int16_t average = 0;
	int count = 0;
	
	HAL_FIFO_AddElement(list, value);	
	HAL_FIFO_RemoveElement(list);
	
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

int16_t HAL_FIFO_GetPitchAverage(int16_t s16_value)
{
	int16_t s16_returnValue;

	s16_returnValue = HAL_FIFO_GetAverage(&fifoPitch, s16_value);
	
	return s16_returnValue;
}

int16_t HAL_FIFO_GetRollAverage(int16_t s16_value)
{
	int16_t s16_returnValue;

	s16_returnValue = HAL_FIFO_GetAverage(&fifoRoll, s16_value);
	
	return s16_returnValue;
}

void HAL_FIFO_GetSonarAverage(uint8_t* u8_distFC, uint8_t* u8_distFL, uint8_t* u8_distFR)
{
	*u8_distFC = HAL_FIFO_GetAverage(&fifoSonarFC, *u8_distFC);
	*u8_distFL = HAL_FIFO_GetAverage(&fifoSonarFC, *u8_distFL);
	*u8_distFR = HAL_FIFO_GetAverage(&fifoSonarFC, *u8_distFR);
}
