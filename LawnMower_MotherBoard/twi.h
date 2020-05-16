//
//  twi.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 28/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef twi_h
#define twi_h

uint8_t TWI_getData(uint8_t, uint8_t);
void TWI_start(void);
void TWI_repeat_start(void);
void TWI_write(uint8_t, uint8_t);
void TWI_write_data(uint8_t);
uint8_t TWI_readACK(void);
uint8_t TWI_readNACK(void);
void TWI_stop(void);

#endif /* twi_h */
