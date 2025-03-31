#include <stdio.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "constant.h"
#include "Initialisation.h"
#include "mower.h"
#include "status.h"
#include "pwm.h"

int main(void) {

    Initialisation();

	FSM_Main_Init();
    FSM_Main();
    return 0 ;
}
/*
	while(!_uBpStart);
	_uBpStart = 0;
	
	while (1) {
		wdt_reset();
		STATUS_updateStatus();
		STATUS_sendStatus();
		
        if(isDocking())
        {
			MOWER_updateBladeState(OFF);
			
			PWM_stop();
			
            if(isCharging())
            {
				_eEtatMower = EN_CHARGE;
				
				if(_uBpForceStart && (_eEtatRain == OFF)) {
					_eEtatMower = TACHE_EN_COURS;
					_uBpStop = 0;
					_uBpForceStart = 0;	
					_eErrorMower = NTR;
					MOWER_leaveDockCharger();
				}
            }
            else if(isTimeToMow() && (_eEtatRain == OFF))
            {
				_eEtatMower = TACHE_EN_COURS;
				_uBpStop = 0;
				_eErrorMower = NTR;	
				MOWER_leaveDockCharger();	
            }
			else
				_eEtatMower = PAS_DE_TACHE_EN_COURS;
        }
		else if(_uBpStart && (!_uBpStop)) {
			_eEtatMower = PAUSE;
			MOWER_updateBladeState(OFF);
			_eErrorMower = NTR;
			
			PWM_stop();
		}
		else if(isEnoughCharged() == -1) {
			MOWER_updateBladeState(OFF);
			_eEtatMower = RETOUR_STATION;
			
			PWM_stop();
		}
        else if(isEnoughCharged() && (!isRaining()) && isTimeToMow() && (!(_uBpStart && _uBpStop)))
        {
			_eEtatMower = TACHE_EN_COURS;
						
            MOWER_startMower();
        }
        else
        {
			_eEtatMower = RETOUR_STATION;
			MOWER_updateBladeState(OFF);
			
            MOWER_goDockCharger();
			
			_uBpStop = 0;
			_uBpStart = 0;
        }
	}
	return 0; // never reached
}
*/
