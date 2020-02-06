#include <stdio.h>
#include <util/delay.h>

#include "Initialisation.h"

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
        else if(isEnougCharged() && !isRaining())
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
