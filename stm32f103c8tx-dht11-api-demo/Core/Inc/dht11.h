/*
 *  dht11.h
 *  Copyright (C) 2020  Marcelo Braga & Jo�o Melga
 *  Created on: feb-03-2021
 *  Institution: UFMG
 *
 *  Version 1.0 - API with the following implemented functions:
 *  void DHT11_init(GPIO_TypeDef* port, uint16_t pin);
 *	uint8_t DHT11_rawread();
 *	float DHT11_temp();
 *	uint8_t DHT11_rh();
 *	float DHT11_dewpoint();
 *	float DHT11_ah();
 *
 *	For more information on the DHT11 sensor, please check the datasheet
 *	provided in the repository
 *
 *	This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#define MAX_READ_TIME 1000 // time in ms
#define MIN_SAMPLING_TIME 1000 // time in ms

#include "stm32f1xx_hal.h" // Modify it according to your microcontroller family.
#include "stdint.h"

/**
 * Used to store setups and data received from DHT11.
 * @typedef {DHT11_struct}
 * @property {*GPIO_TypeDef} port
 * @property {uint16_t} pin
 * @property {uint8_t} tempInt
 * @property {uint8_t} tempDec
 * @property {uint8_t} rhInt
 * @property {uint8_t} rhDec
 * @property {uint8_t} checksum
 */
typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
	uint8_t tempInt;
	uint8_t tempDec;
	uint8_t rhInt;
	uint8_t rhDec;
	uint8_t checksum;
} DHT11_struct;

/**
 * Sets port and pin used to communicate with DHT11.
 * @param  {*GPIO_TypeDef} port - Pin port.
 * @param  {uint16_t} pin - Pin where DHT11 data pin is attatched.
 */
void DHT11_init(GPIO_TypeDef* port, uint16_t pin); //configura a porta e o pino

/**
 * Fills the DHT11_struct and returns 1 if checksum is OK and 0 if NOK.
 * Putting all times together, in worst case - 40bits '1' -, the read time would be 26ms.
 * However, the sampling time should be at least 1 second long.
 * @param  {*GPIO_TypeDef} port - Pin port
 * @param  {uint16_t} pin - Pin where DHT11 data pin is attatched
 */
uint8_t DHT11_rawread();

/**
 * Returns temperature in �C
 * @returns {float} Temperature
 */
float DHT11_temp();


/**
 * Returns relative humidity in %
 * @returns {uint8_t} Relative humidity
 */
uint8_t DHT11_rh();

/**
 * Returns dew point in �C based on Magnus formula
 * @returns {float} Temperature
 */
float DHT11_dewpoint();

/**
 * Returns absolute humidity in g/m� base on formulas available in:
 * https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
 * @returns {float} Temperature
 */
float DHT11_ah();

#endif /* INC_DHT11_H_ */
