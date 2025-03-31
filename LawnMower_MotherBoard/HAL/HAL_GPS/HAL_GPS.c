/*
 * HAL_GPS.c
 *
 *  Created on: 25 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "HAL_GPS.h"
#include "HAL_UART.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
Coordinates gst_latitude;
Coordinates gst_longitude;

uint8_t gu8_minutesGpsAcquisition;
uint8_t gu8_hoursGpsAcquisition;
uint8_t gu8_monthsGpsAcquisition;
uint8_t gu8_daysGpsAcquisition;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPS_initBufferNmea(BufferNmea*);
void HAL_GPS_initDataRmc(DataNmea_RMC*);
uint8_t HAL_GPS_decodeNmeaBuffer(BufferNmea*, DataNmea_RMC*);
uint8_t HAL_GPS_decodeNmeaRmc(BufferNmea*, DataNmea_RMC*);
uint8_t HAL_GPS_getNmeaUart(BufferNmea*, DataNmea_RMC*);
uint8_t HAL_GPS_getNmeaChecksum(char*);
uint8_t HAL_GPS_getNmeaBuffer(BufferNmea*, char);
void HAL_GPS_decodeNmeaForMaster(DataNmea_RMC*);
void HAL_GPS_rmcUtcTime(DataNmea_RMC*);
void HAL_GPS_rmcDate(DataNmea_RMC*);
void HAL_GPS_rmcLatLong(DataNmea_RMC*);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_GPS_Init() 
{
    gu8_minutesGpsAcquisition = 0;
    gu8_hoursGpsAcquisition = 0;
    gu8_monthsGpsAcquisition = 0;
    gu8_daysGpsAcquisition = 0;

    gst_latitude.degrees = 0;
    gst_latitude.minutes = 0;
    gst_latitude.decimalMSB = 0;
    gst_latitude.decimalB = 0;
    gst_latitude.decimalLSB = 0;

    gst_longitude.degrees = 0;
    gst_longitude.minutes = 0;
    gst_longitude.decimalMSB = 0;
    gst_longitude.decimalB = 0;
    gst_longitude.decimalLSB = 0;
}
void HAL_GPS_startGpsAcquisition() {
	uint8_t _bDecodeNmea = 0;

    BufferNmea _pBuffer;
    DataNmea_RMC _pNmeaRmc;
    
    HAL_GPS_initBufferNmea(&_pBuffer);
	HAL_GPS_initDataRmc(&_pNmeaRmc);
	
	_bDecodeNmea = HAL_GPS_getNmeaUart(&_pBuffer, &_pNmeaRmc);
	
	if(_bDecodeNmea) {
		HAL_GPS_decodeNmeaForMaster(&_pNmeaRmc);
	}
}

void HAL_GPS_initBufferNmea(BufferNmea *pBuffer) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        pBuffer->data[i] = 0;
    }
    pBuffer->indice = 0;
    pBuffer->nmea = 0;
}

void HAL_GPS_initDataRmc(DataNmea_RMC *pNmeaRmc) {
    for (unsigned int i = 0; i<sizeof(pNmeaRmc->utcTime); i++) {
        pNmeaRmc->utcTime[i] = 0;
        pNmeaRmc->utcDate[i] = 0;
    }
    for (unsigned int i = 0; i<sizeof(pNmeaRmc->latitude); i++) {
        pNmeaRmc->latitude[i] = 0;
        pNmeaRmc->longitude[i] = 0;
    }
    for (unsigned int i = 0; i<sizeof(pNmeaRmc->speed); i++) {
        pNmeaRmc->speed[i] = 0;
        pNmeaRmc->cap[i] = 0;
        pNmeaRmc->declMagn[i] = 0;
    }
    pNmeaRmc->latitudeDir = 0;
    pNmeaRmc->longitudeDir = 0;
    pNmeaRmc->declMagnDir = 0;
}

uint8_t HAL_GPS_getNmeaUart(BufferNmea *pBuffer, DataNmea_RMC *pNmeaRmc) {
	uint8_t _bTrameNmeaBuffer = 0;
	uint8_t _bDecodeNmeaBuffer = 0;
	uint8_t _cUartRxCounter = 0;
    uint8_t u8_returnValueUART = 0;
	char _tUartRxBuffer[BUFFER_SIZE] = { 0 };
	
    u8_returnValueUART = HAL_UART_ReceptionGPS(_tUartRxBuffer, BUFFER_SIZE);
	
	if(u8_returnValueUART)
    {
		while(_tUartRxBuffer[_cUartRxCounter] != '\n')
        {
			_bTrameNmeaBuffer = HAL_GPS_getNmeaBuffer(pBuffer, _tUartRxBuffer[_cUartRxCounter]);
			_cUartRxCounter++;
		}
        if(_bTrameNmeaBuffer) 
        {
            _bTrameNmeaBuffer = HAL_GPS_getNmeaBuffer(pBuffer, _tUartRxBuffer[_cUartRxCounter]);
            if(_bTrameNmeaBuffer) {
                _bDecodeNmeaBuffer = HAL_GPS_decodeNmeaBuffer(pBuffer, pNmeaRmc);
            }
        }
	}

	return _bDecodeNmeaBuffer;
}

uint8_t HAL_GPS_decodeNmeaBuffer(BufferNmea *pBuffer, DataNmea_RMC *pNmeaRmc) {
	uint8_t _bDecodeRmc = 0;
	char *ptr = &pBuffer->data[3];
	
	if(!(strncmp(ptr, "RMC", 3))) {
		_bDecodeRmc = HAL_GPS_decodeNmeaRmc(pBuffer, pNmeaRmc);
	}
	
	return _bDecodeRmc;
}

uint8_t HAL_GPS_decodeNmeaRmc(BufferNmea *pBuffer, DataNmea_RMC *pNmeaRmc) {
	uint8_t _bDecodeRmc = 0;
	uint8_t _cDataBufferCounter = 0;
	uint8_t _cDataFieldCounter = 0;
	
	pBuffer->nmea = RMC_MESSAGE;
	
	while((pBuffer->data[_cDataBufferCounter] != '*') || (_cDataBufferCounter >= BUFFER_SIZE)) {
		switch(pBuffer->nmea) {
			case RMC_MESSAGE:
				if(pBuffer->data[_cDataBufferCounter] == ',') {
					pBuffer->nmea = RMC_UTC_TIME;
				}
				break;
				
			case RMC_UTC_TIME:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->utcTime[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_STATUS;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_STATUS:
				if(pBuffer->data[_cDataBufferCounter] == ',') {
					pBuffer->nmea = RMC_LAT;
				}
				break;
					
			case RMC_LAT:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->latitude[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_LAT_DIR;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_LAT_DIR:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->latitudeDir = pBuffer->data[_cDataBufferCounter];
				}
				else {
					pBuffer->nmea = RMC_LONG;
				}
				break;
				
			case RMC_LONG:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->longitude[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_LONG_DIR;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_LONG_DIR:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->longitudeDir = pBuffer->data[_cDataBufferCounter];
				}
				else {
					pBuffer->nmea = RMC_SPEED;
				}
				break;
				
			case RMC_SPEED:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->speed[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_CAP;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_CAP:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->cap[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_UTC_DATE;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_UTC_DATE:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->utcDate[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_DECL_MAGN;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_DECL_MAGN:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->declMagn[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_DECL_MAGN_DIR;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_DECL_MAGN_DIR:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->declMagnDir = pBuffer->data[_cDataBufferCounter];
					pBuffer->nmea = RMC_MODE;
					_bDecodeRmc = 1;
				}
				break;
				
			case RMC_MODE:
				break;
				
			default:
				pBuffer->nmea = RMC_MESSAGE;
				_cDataBufferCounter = 0;
				_cDataFieldCounter = 0;
		}
		_cDataBufferCounter++;
	}
	
	return _bDecodeRmc;
}

uint8_t HAL_GPS_getNmeaChecksum(char *dataChecksum) {
    char checksum = 0;
    char values[3];
    uint8_t flagStartData = 0;
    uint8_t indiceStartChecksum = 0;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if ((dataChecksum[i] == '$' || flagStartData) && dataChecksum[i] != '*') {
            if (dataChecksum[i] == '$') {
                i++;
            }
            flagStartData = 1;
            checksum ^= dataChecksum[i];
        }
        else if (dataChecksum[i] == '*') {
            flagStartData = 0;
            indiceStartChecksum = i;
            i = BUFFER_SIZE;
        }
    }
    
    sprintf(values, "%X", checksum);
    if (values[0] == dataChecksum[indiceStartChecksum+1] && values[1] == dataChecksum[indiceStartChecksum+2]) {
        return 1;
    }
    else
        return 0;
}

uint8_t HAL_GPS_getNmeaBuffer(BufferNmea *pBuffer, char byte) {
    uint8_t packetNmea = 0;
    
    if (byte == '$') {
        pBuffer->indice = 0;
        pBuffer->nmea = NMEA_START;
    }
    
    switch (pBuffer->nmea) {
        case NMEA_START:
            if (byte == '$') {
                pBuffer->data[0] = '$';
                pBuffer->nmea = NMEA_G;
            }
            break;
            
        case NMEA_G:
            if (byte == 'G') {
                pBuffer->data[1] = 'G';
                pBuffer->nmea = NMEA_P_A_L_N;
            }
            else {
                pBuffer->nmea = NMEA_START;
            }
            break;
        
        case NMEA_P_A_L_N:
            if (byte == 'P' || byte == 'A' || byte == 'L' || byte == 'N') {
                pBuffer->data[2] = byte;
                pBuffer->indice = 3;
                pBuffer->nmea = NMEA_DATA;
            }
            else {
                pBuffer->nmea = NMEA_START;
            }
            break;
        
        case NMEA_DATA:
            if (byte == '\r') {
                pBuffer->nmea = NMEA_END;
                packetNmea = 1;
            }
            else {
                pBuffer->data[pBuffer->indice++] = byte;
            }
            break;
            
        case NMEA_END:
            if (byte == '\n') {
                pBuffer->data[pBuffer->indice] = 0;
                packetNmea = HAL_GPS_getNmeaChecksum(pBuffer->data);
            }
            break;
            
        default:
            pBuffer->indice = 0;
            pBuffer->nmea = NMEA_START;
            break;
    }
    
    if (pBuffer->indice >= BUFFER_SIZE) {
        pBuffer->indice = 0;
        pBuffer->nmea = NMEA_START;
    }
    
    return packetNmea;
}

void HAL_GPS_decodeNmeaForMaster(DataNmea_RMC *pNmeaRmc) {
	
	HAL_GPS_rmcUtcTime(pNmeaRmc);
	HAL_GPS_rmcDate(pNmeaRmc);
	HAL_GPS_rmcLatLong(pNmeaRmc);
	
}

void HAL_GPS_rmcUtcTime(DataNmea_RMC *pNmeaRmc){
    char tabTemp[3] = {0,0,0};
    
    tabTemp[0] = pNmeaRmc->utcTime[0];
    tabTemp[1] = pNmeaRmc->utcTime[1];
    gu8_hoursGpsAcquisition = (atoi(tabTemp));
    
    tabTemp[0] = pNmeaRmc->utcTime[2];
    tabTemp[1] = pNmeaRmc->utcTime[3];
    gu8_minutesGpsAcquisition = (atoi(tabTemp));
}

void HAL_GPS_rmcDate(DataNmea_RMC *pNmeaRmc){
    char tabTemp[3] = {0,0,0};

    tabTemp[0] = pNmeaRmc->utcDate[0];
    tabTemp[1] = pNmeaRmc->utcDate[1];
    gu8_daysGpsAcquisition = (atoi(tabTemp));

    tabTemp[0] = pNmeaRmc->utcDate[2];
    tabTemp[1] = pNmeaRmc->utcDate[3];
    gu8_monthsGpsAcquisition = (atoi(tabTemp));
}

void HAL_GPS_rmcLatLong(DataNmea_RMC *pNmeaRmc) {
    /*
     * Format lat ddmm.mmmmm -> dd uint8_t ; mm uint8_t ; mmmmm uint32_t  (uint32)MSB << 16 | (uint32)MiSB << 8 | (uint32)LSB
     * Format long dddmm.mmmmm -> ddd uint8_t ; mm uint8_t ; mmmmm uint32_t (uint32)MSB << 16 | (uint32)MiSB << 8 | (uint32)LSB
     */
	char latitudeDegrees[3] = { 0 };
    char latitudeMinutes[3] = { 0 };
    char latitudeDecimal[6] = { 0 };
    char longitudeDegrees[4] = { 0 };
    char longitudeMinutes[3] = { 0 };
    char longitudeDecimal[6] = { 0 };
    uint32_t decimalTemp;
	
    latitudeDegrees[0] = pNmeaRmc->latitude[0];
    latitudeDegrees[1] = pNmeaRmc->latitude[1];
    gst_latitude.degrees = (uint8_t)(atoi(latitudeDegrees));
    
    latitudeMinutes[0] = pNmeaRmc->latitude[2];
    latitudeMinutes[1] = pNmeaRmc->latitude[3];
    gst_latitude.minutes = (uint8_t)(atoi(latitudeMinutes));
    
    latitudeDecimal[0] = pNmeaRmc->latitude[5];
    latitudeDecimal[1] = pNmeaRmc->latitude[6];
    latitudeDecimal[2] = pNmeaRmc->latitude[7];
    latitudeDecimal[3] = pNmeaRmc->latitude[8];
    latitudeDecimal[4] = pNmeaRmc->latitude[9];
    decimalTemp = (uint32_t)(atoi(latitudeDecimal));
    gst_latitude.decimalMSB = (uint8_t)(decimalTemp >> 16);
    gst_latitude.decimalB = (uint8_t)(decimalTemp >> 8);
    gst_latitude.decimalLSB = (uint8_t)(decimalTemp);
    
    longitudeDegrees[0] = pNmeaRmc->longitude[0];
    longitudeDegrees[1] = pNmeaRmc->longitude[1];
    longitudeDegrees[2] = pNmeaRmc->longitude[2];
    gst_longitude.degrees = (uint8_t)(atoi(longitudeDegrees));
    
    longitudeMinutes[0] = pNmeaRmc->longitude[3];
    longitudeMinutes[1] = pNmeaRmc->longitude[4];
    gst_longitude.minutes = (uint8_t)(atoi(longitudeMinutes));
    
    longitudeDecimal[0] = pNmeaRmc->longitude[6];
    longitudeDecimal[1] = pNmeaRmc->longitude[7];
    longitudeDecimal[2] = pNmeaRmc->longitude[8];
    longitudeDecimal[3] = pNmeaRmc->longitude[9];
    longitudeDecimal[4] = pNmeaRmc->longitude[10];
    decimalTemp = (uint32_t)(atoi(longitudeDecimal));
    gst_longitude.decimalMSB = (uint8_t)(decimalTemp >> 16);
    gst_longitude.decimalB = (uint8_t)(decimalTemp >> 8);
    gst_longitude.decimalLSB = (uint8_t)(decimalTemp);
}

uint8_t HAL_GPS_GetHours()
{
    return gu8_hoursGpsAcquisition;
}

uint8_t HAL_GPS_GetMinutes()
{
    return gu8_minutesGpsAcquisition;
}

uint8_t HAL_GPS_GetDays()
{
    return gu8_daysGpsAcquisition;
}

uint8_t HAL_GPS_GetMonths()
{
    return gu8_monthsGpsAcquisition;
}

void HAL_GPS_GetCoordinates(float* pLatitudeCoordinates, float* pLongitudeCoordinates) 
{
    uint32_t tempLatDecimal;
    uint32_t tempLongDecimal;
    
    char tempLat[9] = {0};
    char tempLong[9] = {0};

    tempLatDecimal = ((uint32_t)gst_latitude.decimalMSB << 16) | ((uint32_t)gst_latitude.decimalB << 8) | ((uint32_t)gst_latitude.decimalLSB);
    sprintf(tempLat, "%d.%d",(int)gst_latitude.minutes, (int)tempLatDecimal);
    *pLatitudeCoordinates = (float)gst_latitude.degrees + (atof(tempLat)/60.0);
    
    tempLongDecimal = ((uint32_t)gst_longitude.decimalMSB << 16) | ((uint32_t)gst_longitude.decimalB << 8) | ((uint32_t)gst_longitude.decimalLSB);
    sprintf(tempLong, "%d.%d",(int)gst_longitude.minutes, (int)tempLongDecimal);
    *pLongitudeCoordinates = (float)gst_longitude.degrees + (atof(tempLong)/60.0);
}

void HAL_GPS_GetStructCoordinates(Coordinates* st_latitude, Coordinates* st_longitude)
{
    *st_latitude = gst_latitude;
    *st_longitude = gst_longitude;
}
