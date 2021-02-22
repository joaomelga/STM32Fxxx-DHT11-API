# stm32fxxx-dht11-api

Version 1.0

## Summary
This API controls the communication between an STM32fxxx microcontroller and a DHT11 temperature and humidity sensor.
You can get data from the sensor through the following API funcitons:
  - DHT11_init()
  - DHT11_rawread()
  - DHT11_temp()
  - DHT11_rh()
	- DHT11_ah()
	- DHT11_dewpoint()

This software was designed and tested under STM32 System Workbench IDE + CubeMX. Boards used in development: 
  - STM32F103C8Tx board, "Blue Pill" : https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html;
  - STM32F407VET6 board, "Black Board" : https://stm32-base.org/boards/STM32F407VET6-STM32-F4VE-V2.0.html.

## Setup
To setup and use the API properly you must accomplish the following steps:
  - Set your clock tree and TIM1 (or other timer) to generate an interruption for each us (1MHz frequency);
  - Copy dht11.h and dht11.c to Inc and Src folders respectively;
  - In Inc/dht11.h, include the properly HAL library according to your microcontroller family;
  - A DHT_struct variable must be declared in your main function, named Dstruct;
  - The DHT11_init() function should be called once in your setup.

## API demo instructions
You will find attached two API demos in this repository, one for the stm32f103c8t6 microcontroller and another for the stm32f407vet6 microcontroller.
They are structered around a Finite State Machine that changes between the API functions every 3500 milliseconds, showing function results in a 128x64 OLED Display (Display Model: https://www.eletrogate.com/display-oled-0-96-i2c-azul, Library published by Alexander Lutsai, found in: https://github.com/SL-RU/stm32libs). The display uses a I2C interface, which requires two pin connections:
  - Pin xxx : I2C_SCL, Pin xxx : I2C_SDA for the stm32f103c8tx-dht11-api-demo;
  - Pin PB8 : I2C_SCL, Pin PB7 : I2C_SDA for the stm32f407VETx-dht11-api-demo.

The DHT11 sensor is connected to the PA1 pin in both boards.

## Recommendations
The API will return 0xFF (255 in decimal) in temperature/humidity values if reading of the sensor was not done correctly. If that is the case, you may try:
  - Check if your timer was configured correcly. Keep in mind that higher frequency timers showed better results in the development of this API;  
  - New readings should not be done in intervals less than 1 second, as specified in the DHT11 Datasheet. In that case, the API design garantees that the Dstruct variable will remain with values of the last call;
  - Check your power supply.

## Licence
This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the Licence, or (at your option) any later version. 
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Se the GNU General Public Licence for more details.
For more information, read the LICENCE file available in the project root.

Authors: Marcelo Braga & João Melga 
Created on: feb-03-2021
