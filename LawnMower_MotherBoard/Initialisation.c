//
//  Initialisation.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 25/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include "Initialisation.h"

#define F_CPU 8000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void Initialisation()
{
    InitIO();
    InitPWM();
    InitInterrupt();
    InitI2C();
    InitUART(MYUBRR);
}

void InitIO()
{
    DDRC |= (1<<DDC0) | (1<<DDC1); //LED Moteur - ON : PINC0 ; OFF : PINC1
    DDRC |= (1<<DDC2) | (1<<DDC3); //LED Vitesse - 1 : PINC2 ; 2 : PINC3
    DDRC |= (1<<DDC4) | (1<<DDC5); //LED Sens - UP : PINC4 ; DOWN : PINC5
    DDRD |= (1<<DDD3); //Commande Lumiere - ON/OFF : PIND3
    DDRD |= (1<<DDD4) | (1<<DDD5); //Commande Moteur droite - PWM : PIND5 ; Normal :PIND4
    DDRD |= (1<<DDD6) | (1<<DDD7); //Commande Moteur gauche - PWM : PIND6 ; Normal : PIND7
    
    PORTC &= ~(1<<PORTC0) & ~(1<<PORTC1) & ~(1<<PORTC2) & ~(1<<PORTC3) & ~(1<<PORTC4) & ~(1<<PORTC5);
    PORTD &= ~(1<<PORTD3);
    PORTD |= (1<<PORTD4) | (1<<PORTD7); // PNP a 1 pour les desactiver
    PORTD &= ~(1<<PORTD5) & ~(1<<PORTD6);
}

void InitInterrupt()
{
    sei();
}

void InitPWM()
{
/***** Moteur 1 - Droit *****/
    TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); // Fast PWM
    TCCR0B |= (1<<CS00); // No Prescale
    
    OCR0A = 0x00; // Marche Avant
    OCR0B = 0x00; // Marche Arrière
    
/***** Moteur 2 - Gauche *****/
    TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); // Fast PWM
    TCCR2B |= (1<<CS20); // No Prescale
    
    OCR2A = 0x00; // Marche Avant
    OCR0B = 0x00; // Marche Arrière
}

void InitI2C()
{
    TWBR = 32; //TWBR  = ((F_CPU / SCL_CLK) – 16) / 2
    TWCR = (1<<TWIE) | (1<<TWEN);
}

void InitUART(unsigned int ubrr)
{
/***** UART BaudRate *****/
    UBRR0H = (unsigned char) (ubrr>>8);
    UBRR0L = (unsigned char) ubrr;
    
/***** Autoriser Transmition et Reception *****/
    UCSR0B |= (1<<RXCIE0) | (1<<TXEN0) | (1<<RXEN0);
    UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
    
}
