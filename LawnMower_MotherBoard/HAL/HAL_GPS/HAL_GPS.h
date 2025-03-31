/*
 * HAL_GPS.h
 *
 *  Created on: 25 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_GPS_HAL_GPS_H_
#define HAL_HAL_GPS_HAL_GPS_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define BUFFER_SIZE_MAX 100
#define BUFFER_SIZE 80

typedef enum {
    NMEA_START, //$
    NMEA_G,
    NMEA_P_A_L_N, // GPS, GALILEO, GLONNASS
    NMEA_DATA,
    NMEA_END, //\r
}PacketNmea;

typedef enum {
    RMC_MESSAGE,
    RMC_UTC_TIME,
    RMC_STATUS,
    RMC_LAT,
    RMC_LAT_DIR,
    RMC_LONG,
    RMC_LONG_DIR,
    RMC_SPEED,
    RMC_CAP,
    RMC_UTC_DATE,
    RMC_DECL_MAGN,
    RMC_DECL_MAGN_DIR,
    RMC_MODE
}PacketRmc;

typedef struct {
    char data[BUFFER_SIZE];
    uint8_t indice;
    unsigned char nmea;
}BufferNmea;

/*** Trame RMC ***/
/* $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68 */
/* type,UTC hhmmss, Alerte, Lat ddmm.mm, Dir N/S, Lon dddmm.mm, Dir E/W, vitesse Noeud, cap, Date jjmmaa, declinaison magnétique ddd.d, Dir E/W, checksum */
/*****************/

typedef struct {
    char utcTime[7];
    char latitude[15];
    char latitudeDir;
    char longitude[15];
    char longitudeDir;
    char speed[6];
    char cap[6];
    char utcDate[7];
    char declMagn[6];
    char declMagnDir;
}DataNmea_RMC;

typedef struct {
    uint8_t degrees;
    uint8_t minutes;
    uint8_t decimalMSB;
    uint8_t decimalB;
    uint8_t decimalLSB;
}Coordinates;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPS_Init(void);
void HAL_GPS_startGpsAcquisition(void);
uint8_t HAL_GPS_GetHours(void);
void HAL_GPS_GetCoordinates(float* pLatitudeCoordinates, float* pLongitudeCoordinates);
void HAL_GPS_GetStructCoordinates(Coordinates* st_latitude, Coordinates* st_longitude);

#endif /* HAL_HAL_GPS_HAL_GPS_H_ */
