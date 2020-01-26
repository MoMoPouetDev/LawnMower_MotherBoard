#include <stdio.h>

#include <util/delay.h>

#include "Initialisation.h"

#define F_CPU 8000000UL

int main(void) {

    Initialisation();

    printf("helloWorld");
	while (1) {
	}
	return 0; // never reached
}
