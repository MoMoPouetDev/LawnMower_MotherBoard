#include <stdio.h>
#include <util/delay.h>

#include "Initialisation.h"
#include "mower.h"

int main(void) {

    Initialisation();

	while (1) {
        if(isDocking())
        {
            if(isCharging())
            {
                //Envoie periodique BLE "En Charge" Avec valeur batterie
            }
            else if(isTimeToMow())
            {
                leaveDockCharger();
            }
        }
        else if(isEnoughCharged() && !isRaining())
        {
            startMower();
        }
        else
        {
            goDockCharger();
        }
	}
	return 0; // never reached
}
