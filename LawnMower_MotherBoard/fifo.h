/*
 * fifo.h
 *
 * Created: 18/07/2021 11:33:09
 *  Author: morga
 */ 


#ifndef FIFO_H_
#define FIFO_H_

typedef struct Element Element;
struct Element {
	int value;
	Element *nextElement;
};

typedef struct Fifo Fifo;
struct Fifo {
	Element *firstElement;
};

Fifo fifoSonarFC;
Fifo fifoSonarFL;
Fifo fifoSonarFR;
Fifo fifoLeftWire;
Fifo fifoRightWire;
Fifo fifoPitch;
Fifo fifoRoll;

void FIFO_initList(Fifo *, int);
void FIFO_addElement(Fifo *, int);
void FIFO_removeElement(Fifo *);
int FIFO_getAverage(Fifo *, int);

#endif /* FIFO_H_ */