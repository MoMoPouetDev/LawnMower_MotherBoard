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
    NTR = 0x08,
    BLOCKED_MOWER = 0x09,
    DETECTED_RAIN = 0x0A,
    WIRE_NOT_DETECTED = 0x0B,
    LOW_BATTERY = 0x0C,
    VERY_LOW_BATTERY = 0x0D,
    EMPTY_BATTERY = 0x0E
}ErrorMower;
ErrorMower _eErrorMower;

typedef enum {
    UNKNOWN_COMMAND = 0x10,
    START = 0x11,
    STOP = 0x12,
    FORCE_START = 0x13
}CommandMower;
CommandMower _eCommandMower;

void STATUS_updateStatus(void);
void STATUS_updateStatusLed(void);
void STATUS_updateStatusError(void);
void STATUS_sendStatus(void);
void STATUS_receivedStatus(void);

#endif /* uart_h */
