/*
 * initialisation.c
 *
 * Created: 20/10/2021 00:04:54
 *  Author: morgan
 */ 

#include "sam.h"

void Initialisation() {
	INIT_io();
	INIT_variable();
	INIT_pwm();
	INIT_timer();
	INIT_twi();
	INIT_uart();
	INIT_adc();
	INIT_wdt();
	INIT_interrupt();
	INIT_compass();
	INIT_accel();
	INIT_ble();
}

void INIT_io() {
	/*** Bouton Poussoir ***/
	//STOP
	PORT->Group[0].PINCFG[10].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA10;
	//START
	PORT->Group[0].PINCFG[11].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA11;
	
	/*** Bumper ***/
	//PA17 PA18 PA19
	PORT->Group[0].PINCFG[17].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA17;
	PORT->Group[0].PINCFG[18].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA18;
	PORT->Group[0].PINCFG[19].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA19;
	
	/*** Sonar ***/
	// echo PA22 PA23 PA24
	PORT->Group[0].PINCFG[22].bit.INEN = 1;
	PORT->Group[0].PINCFG[23].bit.INEN = 1;
	PORT->Group[0].PINCFG[24].bit.INEN = 1;
	
	// trigger PA25 PA27 PA28
	REG_PORT_DIRSET0 |= PORT_PA25 | PORT_PA27 | PORT_PA28;
	REG_PORT_OUTCLR0 |= PORT_PA25 | PORT_PA27 | PORT_PA28;
	
	/*** LED ***/
	// PB22 PB23 PA30 PA31 PB02 PB03
	REG_PORT_DIRSET0 |= PORT_PA30 | PORT_PA31;
	REG_PORT_DIRSET1 |= PORT_PB22 | PORT_PB23 | PORT_PB02 | PORT_PB03;
	REG_PORT_OUTCLR0 |= PORT_PA30 | PORT_PA31;
	REG_PORT_OUTCLR1 |= PORT_PB22 | PORT_PB23 | PORT_PB02 | PORT_PB03;
	
	/*** PWM ***/
	/*** Moteur 1 ***/
	// PWM -> PA07; PA12 PA13
	REG_PORT_DIRSET0 |= PORT_PA07 | PORT_PA12 | PORT_PA13;
	REG_PORT_OUTCLR0 |= PORT_PA07 | PORT_PA12 | PORT_PA13;
	PORT->Group[0].PINCFG[7].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_F;
	
	/*** Moteur 2 ***/
	// PWM -> PA16; PA20 PA21
	REG_PORT_DIRSET0 |= PORT_PA16 | PORT_PA20 | PORT_PA21;
	REG_PORT_OUTCLR0 |= PORT_PA16 | PORT_PA20 | PORT_PA21;
	PORT->Group[0].PINCFG[16].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[9].bit.PMUXE = PORT_PMUX_PMUXE_F;
	
	/*** Moteur Lame ***/
	// PB10
	REG_PORT_DIRSET1 |= PORT_PB10;
	REG_PORT_OUTCLR1 |= PORT_PB10;
	
	/*** ADC ***/
	// ADC -> PA02 .. PA06
	PORT->Group[0].PINCFG[2].bit.INEN = 1;
	PORT->Group[0].PINCFG[3].bit.INEN = 1;
	PORT->Group[0].PINCFG[4].bit.INEN = 1;	
	PORT->Group[0].PINCFG[5].bit.INEN = 1;
	PORT->Group[0].PINCFG[6].bit.INEN = 1;
		
	/*** I2C ***/
	//PA08 PA09
	PORT->Group[0].PINCFG[8].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[5].bit.PMUXE = PORT_PMUX_PMUXE_C;
	PORT->Group[0].PINCFG[9].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[5].bit.PMUXO = PORT_PMUX_PMUXO_C;
	
	/*** UART ***/
	/*** BLE ***/
	//PA12 PA13
	PORT->Group[0].PINCFG[12].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[7].bit.PMUXE = PORT_PMUX_PMUXE_C;
	PORT->Group[0].PINCFG[13].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[7].bit.PMUXO = PORT_PMUX_PMUXO_C;
	
	/*** GPS¨***/
	//PA14 PA15
	PORT->Group[0].PINCFG[14].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[8].bit.PMUXE = PORT_PMUX_PMUXE_C;
	PORT->Group[0].PINCFG[15].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[8].bit.PMUXO = PORT_PMUX_PMUXO_C;
		
}

void INIT_interrupt() {
	
}

void INIT_pwm() {
	
}