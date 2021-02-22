/*
 *  dht11.c
 *  Copyright (C) 2020  Marcelo Braga & João Melga
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

#include "dht11.h"
#include "stdint.h"

#define htim htim1 // You must set your clock tree and TIM1 (or other timer) to generate an interruption for each us (1MHz frequency)

extern TIM_HandleTypeDef htim;
extern DHT11_struct Dstruct;

uint32_t lastReadTimestamp = 0, timeElapsedSinceLastRead = MIN_SAMPLING_TIME;

//Auxiliary functions
void updateTimeElapsed();
void delay_us(uint16_t us);
void Set_Output(GPIO_TypeDef *port, uint16_t pin);
void Set_Input(GPIO_TypeDef *port, uint16_t pin);
uint8_t DHT11_setup();

void DHT11_init(GPIO_TypeDef* port, uint16_t pin) {
  Dstruct.port = port;
  Dstruct.pin = pin;
}

uint8_t DHT11_rawread() {
  if (DHT11_setup()) {
    uint8_t i, j, value = 0;
    uint8_t values[5];

    // Receives five 8-bits values (bit '0' = High for 26us, and bit '1' = High 1 for 80us)
    for (j = 0; j < 5; j++) {
      for (i = 0; i < 8; i++) {
        while (!(HAL_GPIO_ReadPin(Dstruct.port, Dstruct.pin))
            && timeElapsedSinceLastRead <= MAX_READ_TIME) {
          updateTimeElapsed();
        }

        delay_us(47);

        if (!(HAL_GPIO_ReadPin(Dstruct.port, Dstruct.pin)))
          value &= ~(1 << (7 - i)); // if 0, read bit '0'
        else
          value |= (1 << (7 - i)); // if 1, read bit '1'

        while ((HAL_GPIO_ReadPin(Dstruct.port, Dstruct.pin))
            && timeElapsedSinceLastRead <= MAX_READ_TIME) {
          updateTimeElapsed();
        }
      }

      values[j] = value;
    }

    Dstruct.rhInt = values[0];
    Dstruct.rhDec = values[1];
    Dstruct.tempInt = values[2];
    Dstruct.tempDec = values[3];
    Dstruct.checksum = values[4];

    /*
     if(!(Dstruct.checksum == Dstruct->temperature + Dstruct->humidity)){
     return -1;
     else
     return 1;
     */

    return 1;
  } else
    return 0;

}

float DHT11_temp() {
  updateTimeElapsed();

  if (timeElapsedSinceLastRead > MIN_SAMPLING_TIME) {
    DHT11_rawread();
  }

  return (float) Dstruct.tempInt + ((float) Dstruct.tempDec * 0.1);
}

uint8_t DHT11_rh() {
  updateTimeElapsed();

  if (timeElapsedSinceLastRead > MIN_SAMPLING_TIME) {
    DHT11_rawread();
  }

  return Dstruct.rhInt;
}

float DHT11_ah() {
  updateTimeElapsed();

  if (timeElapsedSinceLastRead > MIN_SAMPLING_TIME) {
    DHT11_rawread();
  }

  float aux, ah;

  aux = exp((17.67 * Dstruct.tempInt) / (243.5 + Dstruct.tempInt));
  ah = 6.112 * aux * Dstruct.rhInt * 2.1674 / (273.15 + Dstruct.tempInt);

  return ah;
}

float DHT11_dewpoint() {
  updateTimeElapsed();

  if (timeElapsedSinceLastRead > MIN_SAMPLING_TIME) {
    DHT11_rawread();
  }

  float gama, dp;

  gama = log(Dstruct.rhInt / 100.0)
      + (17.67 * Dstruct.tempInt) / (243.5 + Dstruct.tempInt);
  dp = (243.5 * gama) / (17.67 - gama);

  return dp;
}

/**
 * Auxiliary function to setup DHT11 via "handshake"
 * @returns {uint8_t} 1 = OK, 0 = NOK
 */
uint8_t DHT11_setup() {
  lastReadTimestamp = HAL_GetTick(), timeElapsedSinceLastRead = 0;

  // To initialize communication, send 0 for 18ms
  Set_Output(Dstruct.port, Dstruct.pin);
  HAL_GPIO_WritePin(Dstruct.port, Dstruct.pin, 1);
  delay_us(1000);
  HAL_GPIO_WritePin(Dstruct.port, Dstruct.pin, 0);
  delay_us(20000);
  HAL_GPIO_WritePin(Dstruct.port, Dstruct.pin, 1);
  delay_us(20);
  Set_Input(Dstruct.port, Dstruct.pin);

  // Await sensor response (it sends 0 for 80us and, just before that, sends 1 for 80us)
  uint8_t DHTStatus = 0;
  delay_us(40);
  if (!(HAL_GPIO_ReadPin(Dstruct.port, Dstruct.pin))) {
    delay_us(80);
    if ((HAL_GPIO_ReadPin(Dstruct.port, Dstruct.pin)))
      DHTStatus = 1;
    else
      return 0;
  }

  while ((HAL_GPIO_ReadPin(Dstruct.port, Dstruct.pin))
      && timeElapsedSinceLastRead <= MAX_READ_TIME)
    updateTimeElapsed();
  return 1;
}

/**
 * Auxiliary function to update time elapsed
 */
void updateTimeElapsed() {
  timeElapsedSinceLastRead = HAL_GetTick() - lastReadTimestamp;
}

/**
 * Auxiliary function to generate delays in us
 * @param {uint16_t} us - Time in us to delay.
 */
void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim, 0);  // Set counter to 0
  while (__HAL_TIM_GET_COUNTER(&htim) < us)
    ; // wait...
}

/**
 * Auxiliary function to set pin as output
 * @param {*GPIO_TypeDef} port
 * @param {uint16_t} pin
 */
void Set_Output(GPIO_TypeDef *port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/**
 * Auxiliary function to set pin as input
 * @param {*GPIO_TypeDef} port
 * @param {uint16_t} pin
 */
void Set_Input(GPIO_TypeDef *port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

