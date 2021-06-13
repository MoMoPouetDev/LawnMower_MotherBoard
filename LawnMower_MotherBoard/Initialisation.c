//
//  Initialisation.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 25/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <util/delay.h>

#include "constant.h"
#include "Initialisation.h"
#include "status.h"
#include "twi.h"
#include "uart.h"

void Initialisation()
{
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

void INIT_io()
{
/***** PORT B *****/
    DDRB = 0x00;
    //DDRB |= (0<<DDB0); // Bouton Poussoir Stop
    //DDRB |= (1<<DDB1) | (1<<DDB2) | (1<<DDB4); // Bumper Left - Center - Right
	DDRB |= (1<<DDB3); // PWM Moteur 2
	DDRB |= (1<<DDB5); // Commande Moteur Lame Enable 
    //DDRB |= (1<<DDB6) | (1<<DDB7); // XTAL
    
    PORTB = 0x00;
    PORTB |= (1<<PORTB0); // Pull-Up Bouton Poussoir
    PORTB |= (1<<PORTB1) | (1<<PORTB2) | (1<<PORTB4); // Bumper Left - Center - Right Pull Up
    //PORTB |= (1<<PORTB3); // Moteur 2 Avant
	//PORTB |= (1<<PORTB5); // Moteur Lame Enable
    //PORTB |= (1<<PORTB6); // XTAL
    //PORTB |= (1<<PORTB7); // XTAL
    
/***** PORT C *****/
    DDRC = 0x00;
    //DDRC |= (1<<DDC0) | (1<<DDC1); // ADC - Detection cable droite et gauche
    //DDRC |= (1<<DDC2) | (1<<DDC3); // TBD
    //DDRC |= (1<<DDC4) | (1<<DDC5); // Config I2C SDA - SCL
    //DDRC |= (1<<DDC6); // TBD
    
    PORTC = 0x00;
    //PORTC &= ~(1<<PORTC0) & ~(1<<PORTC1); // ADC - No Pull-Up
    PORTC |= (1<<PORTC2) | (1<<PORTC3); // TBD Pull Up
    //PORTC &= ~(1<<PORTC4) & ~(1<<PORTC5); // I2C - Force à 0
    PORTC |= (1<<PORTC6); // TBD - Pull-Up
    
/***** PORT D *****/
    DDRD = 0x00;
    DDRD |= (1<<DDD1); //| (0<<DDD0); // UART - TXD - RXD
    DDRD |= (1<<DDD2) | (1<<DDD3); // Commande Avant Moteur 2 - Commande Arriere Moteur 2
    DDRD |= (1<<DDD4) | (1<<DDD5) | (1<<DDD6); // Commande Avant Moteur 1 - Commande Arriere Moteur 1 - PWM Moteur 1
    //DDRD |= (0<<DDD7); // Bouton Poussoir Start
    
    PORTD = 0x00;
    PORTD |= (1<<PORTD0); //| (1<<PORTD1); // UART - RX Pull-Up - TX
    //PORTD |= (1<<PORTD2) | (1<<PORTD3); // Commande Avant Moteur 2 - Commande Arriere Moteur 2 - Force à 0
    //PORTD |= (1<<PORTD4) | (1<<PORTD5) | (1<<PORTD6); // Commande Avant Moteur 1 - Commande Arriere Moteur 1 - PWM Moteur 1
    PORTD |= (1<<PORTD7); // Pull-Up Bouton Poussoir
}

void INIT_interrupt()
{
    PCICR |= (1<<PCIE2) | (1<<PCIE0); // Activation des Interruptions sur PCINT[23:16] et PCINT[7:0]
    PCMSK2 |= (1<<PCINT23); // Activation des Interruptions sur PCINT23
    PCMSK0 |= (1<<PCINT0); // Activation des Interruptions sur PCINT0
    sei();
}

void INIT_pwm()
{
/***** Moteur 1 - Gauche *****/
    TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00); // Fast PWM
    TCCR0B |= (1<<CS00); // No Prescale
    
    OCR0A = 0x00;
    
/***** Moteur 2 - Droit *****/
    TCCR2A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); // Fast PWM
    TCCR2B |= (1<<CS20); // No Prescale
    
    OCR2A = 0x00;
}

void INIT_timer()
{
	//TCCR1B |= (1<<CS10); // No Prescale
}

void INIT_twi()
{
	TWSR = 0;
    TWBR  = (( F_CPU  / SCL_CLOCK ) - 16 ) / 2; //- 400kHz
}

void INIT_wdt()
{
	cli();
	wdt_reset();
	WDTCSR = (1<<WDCE) | (1<<WDE);
	WDTCSR = (1<<WDIE) | (1<<WDP2);
	sei();
}

void INIT_uart()
{
/***** UART BaudRate *****/
    UBRR0H = (unsigned char) (BAUD_PRESCALE>>8);
    UBRR0L = (unsigned char) BAUD_PRESCALE;
    
/***** Autoriser Transmition et Reception *****/
    UCSR0B = (1<<RXCIE0) | (1<<TXEN0) | (1<<RXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
    
}

void INIT_adc()
{
    ADMUX |= (1<<REFS0);
    ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // Enable ADC, Interrup et 64 prescale
}

void INIT_compass()
{
	TWI_start();
	TWI_write(ADDR_SLAVE_COMPASS, TW_WRITE);
	TWI_write_data(0x00);
	TWI_write_data(0x70);
	TWI_write_data(0xA0);
	TWI_write_data(0x00);
	_delay_ms(20);
	TWI_stop();
	_delay_ms(20);
}

void INIT_accel()
{
	TWI_start();
	TWI_write(ADDR_SLAVE_ACCELEROMETER, TW_WRITE);
	TWI_write_data(0x2D);
	TWI_write_data(0x08);
	_delay_ms(20);
	TWI_stop();
	_delay_ms(20);
}

void INIT_ble()
{
	char commandAT[] = "AT",
		commandRole[] = "AT+ROLE0",
		commandUuid[] = "AT+UUID0xFFE0",
		commandChar[] = "AT+CHAR0xFFE1",
		commandName[] = "AT+NAMEMower";
		
	UART_sendCommand(commandAT);
	UART_sendCommand(commandRole);
	UART_sendCommand(commandUuid);
	UART_sendCommand(commandChar);
	UART_sendCommand(commandName);
}

void INIT_variable()
{
	_eEtatRain = OFF;
	_eEtatMower = UNKNOWN_ETAT;
	_eErrorMower = NTR;
	_uBpStop = 0;
	_uBpStart = 0;
	_uBpForceStart = 0;
	_uWireReached = 0;
	_uFlagWatchdog = 0;
}

