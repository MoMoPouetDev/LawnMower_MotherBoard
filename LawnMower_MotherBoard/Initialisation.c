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

#include "constant.h"
#include "status.h"
#include "twi.h"
#include "Initialisation.h"

void Initialisation()
{
    INIT_io();
    INIT_variable();
    INIT_pwm();
    INIT_twi();
    INIT_uart();
    INIT_adc();
    INIT_interrupt();
}

void INIT_io()
{
/***** PORT B *****/
    DDRB = 0x00;
    //DDRB |= (0<<DDB0); // Bouton Poussoir Stop
    DDRB |= (1<<DDB1) | (1<<DDB2) | (1<<DDB4) | (1<<DDB5); // LED d'état Vert - Orange - Rouge - Jaune
    DDRB |= (1<<DDB3); // Commande PWM Avant Moteur 2
    DDRB |= (1<<DDB6) | (1<<DDB7); // Commande Moteur Lame Enable - ON
    
    PORTB = 0x00;
    PORTB |= (1<<PORTB0); // Pull-Up Bouton Poussoir
    // PORTB &= ~(1<<PORTB1) & ~(1<<PORTB2) & ~(1<<PORTB4) & ~(1<<PORTB5); // Force à 0 LED
    // PORTB |= (1<<PORTB3); // Moteur 2 Avant
    // PORTB |= (1<<PORTB6); // Enable Moteur Lame
    // PORTB |= (1<<PORTB7); // Moteur Lame
    
/***** PORT C *****/
    DDRC = 0x00;
    //DDRC |= (1<<DDC0) | (1<<DDC1); // ADC - Detection cable droite et gauche
    DDRC |= (1<<DDC2) | (1<<DDC3); // LED d'état Jaune
    DDRC |= (1<<DDC4) | (1<<DDC5); // Config I2C SDA - SCL
    //DDRC |= (1<<DDC6); // TBD
    
    PORTC = 0x00;
    //PORTC &= ~(1<<PORTC0) & ~(1<<PORTC1); // ADC - No Pull-Up
    //PORTC |= (1<<PORTC2) | (1<<PORTC3); // LED d'état Jaune
    //PORTC &= ~(1<<PORTC4) & ~(1<<PORTC5); // I2C - Force à 0
    PORTC |= (1<<PORTC6); // TBD - Pull-Up
    
/***** PORT D *****/
    DDRD = 0x00;
    DDRD |= (1<<DDD1); //| (0<<DDD0); // UART - TXD - RXD
    DDRD |= (1<<DDD2) | (1<<DDD3); // Commande Enable - PWM Arriere Moteur 2
    DDRD |= (1<<DDD4) | (1<<DDD5) | (1<<DDD6); // Commande Enable - PWM Arriere - PWM Avant Moteur 1
    //DDRD |= (0<<DDD7); // Bouton Poussoir Start
    
    PORTD = 0x00;
    PORTD |= (1<<PORTC0); //| (1<<PORTC1); // UART - RX Pull-Up - TX
    //PORTD |= (1<<PORTD2) | (1<<PORTD3); // Commande Enable - PWM Arriere Moteur 2 - Force à 0
    //PORTD |= (1<<PORTD4) | (1<<PORTD5) | (1<<PORTD6); // Commande Enable - PWM Arriere - PWM Avant Moteur 1
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
    TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); // Fast PWM
    TCCR0B |= (1<<CS00); // No Prescale
    
    OCR0A = 0x00; // Marche Avant
    OCR0B = 0x00; // Marche Arrière
    
/***** Moteur 2 - Droit *****/
    TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); // Fast PWM
    TCCR2B |= (1<<CS20); // No Prescale
    
    OCR2A = 0x00; // Marche Avant
    OCR2B = 0x00; // Marche Arrière
}

void INIT_twi()
{
    TWBR = 2; //TWBR  = ((F_CPU / SCL_CLK) – 16) / 2 - 400kHz
    TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

void INIT_uart()
{
/***** UART BaudRate *****/
    UBRR0H = (BAUD_PRESCALE>>8);
    UBRR0L = BAUD_PRESCALE;
    
/***** Autoriser Transmition et Reception *****/
    UCSR0B |= (1<<RXCIE0) | (1<<TXEN0) | (1<<RXEN0);
    UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
    
}

void INIT_adc()
{
    ADMUX |= (1<<REFS0);
    ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // Enable ADC, Interrup et 64 prescale
}

void INIT_variable()
{
    _eEtatBlade = OFF;
	_eEtatRain = OFF;
	_eEtatMower = UNKNOWN_ETAT;
	_eErrorMower = NTR;
	_eCommandMower = UNKNOWN_COMMAND;
    _eDataAdress = UNKNOWN_DATA;
	
    _uCharge = 0;
    _uBattery = 0;
    _uUnderTheRain = 0;
    _uTimeToMow = 0;
    _uDock = 0;
	_uBpStop = 0;
	_uBpStart = 0;
	_uBpForceStart = 0;
	_uWireReached = 0;
}

