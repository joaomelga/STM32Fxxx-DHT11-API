# stm32fxxx-dht11-api

Version 1.0

## Summary
This API controls the communication between an STM32fxxx microcontroller and a DHT11 temperature and humidity sensor.
You can get data from the sensor through the following api funcitons:
  - DHT11_init()
  - DHT11_rawread()
  - DHT11_temp()
  - DHT11_rh()
	- DHT11_ah()
	- DHT11_dewpoint()

This software was designed and tested under STM32 System Workbench IDE + CubeMX.

## Setup
To setup and use the api properly you must accomplish the following steps:
  - Set your clock tree and TIM1 (or other timer) to generate an interruption for each us (1MHz frequency);
  - Copy dht11.h and dht11.c to Inc and Src folders respectively;
  - In Inc/dht11.h, include the properly HAL library according to your microcontroller family.

## Licence
This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the Licence, or (at your option) any later version. 
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Se the GNU General Public Licence for more details.
For more information, read the LICENCE file available in the project root.

Authors: Marcelo Braga & João Melga 
Created on: feb-03-2021
