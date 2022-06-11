/*
 * fifo.c
 *
 * Created: 18/07/2021 11:30:26
 *  Author: morga
 */ 

#include <stdio.h>
#include <stdlib.h>

#include "constant.h"
#include "fifo.h"

void FIFO_initList(Fifo *list, int value) {
	Element *element = malloc(sizeof(*element));
	if ((list == NULL) || (element == NULL)) {
		exit(1);
	}
	
	element->value = value;
	element->nextElement = NULL;
	
	list->firstElement = element;
	
	for (int i = 0; i < FIFO_SIZE - 1; i++)
	{
		FIFO_addElement(list, value);
	}
}

void FIFO_addElement(Fifo *list, int value) {
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

void FIFO_removeElement(Fifo *list) {
	if (list == NULL)
	{
		exit(1);
	}
	
	Element *removedElement = list->firstElement;
	list->firstElement = list->firstElement->nextElement;
	free(removedElement);
}

int FIFO_getAverage(Fifo *list, int value) {
	int average = 0;
	int count = 0;
	
	FIFO_addElement(list, value);	
	FIFO_removeElement(list);
	
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