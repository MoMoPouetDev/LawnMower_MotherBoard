//
//  status.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 21/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef status_h
#define status_h

typedef enum {
    UNKNOWN_ETAT = 0x00,
    TACHE_EN_COURS = 0x01,
    RETOUR_STATION = 0x02,
    EN_CHARGE = 0x03,
    PAS_DE_TACHE_EN_COURS = 0x04,
    PAUSE = 0x05
}EtatMower;
EtatMower _eEtatMower;

typedef enum {
    NTR = 0x10,
    BLOCKED_MOWER = 0x20,
    DETECTED_RAIN = 0x30,
    WIRE_NOT_DETECTED = 0x40,
    LOW_BATTERY = 0x50,
    VERY_LOW_BATTERY = 0x60,
    EMPTY_BATTERY = 0x70
}ErrorMower;
ErrorMower _eErrorMower;

typedef enum {
    UNKNOWN_COMMAND = 0x30,
    START = 0x31,
    STOP = 0x32,
    FORCE_START = 0x33,
	DOCK_ON = 0x34,
	DOCK_OFF = 0x35
}CommandMower;
CommandMower _eCommandMower;

void STATUS_updateStatus(void);
void STATUS_updateStatusLed(uint8_t*);
void STATUS_updateStatusError(uint8_t*);
void STATUS_sendStatus(void);
void STATUS_receivedStatus(void);

#endif /* uart_h */
