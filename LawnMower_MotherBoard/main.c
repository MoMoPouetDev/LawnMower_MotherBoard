#include <stdio.h>
#include <util/delay.h>

#include "FSM_Main.h"

int main(void) 
{
	FSM_Main_Init();
    FSM_Main();
	
    return 0 ;
}

