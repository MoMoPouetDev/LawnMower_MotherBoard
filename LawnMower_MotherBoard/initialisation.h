/*
 * initialisation.h
 *
 * Created: 20/10/2021 00:15:29
 *  Author: morgan
 */ 


#ifndef INITIALISATION_H_
#define INITIALISATION_H_

void Initialisation(void);
void INIT_io(void);
void INIT_pwm(void);
void INIT_timer(void);
void INIT_interrupt(void);
void INIT_twi(void);
void INIT_uart(void);
void INIT_adc(void);
void INIT_variable(void);
void INIT_wdt(void);
void INIT_compass(void);
void INIT_accel(void);
void INIT_ble(void);

#endif /* INITIALISATION_H_ */