# stm32fxxx-dht11-api

Copyright (C) 2020  Marcelo Braga & João Melga

Created on: feb-03-2021

Institution: Universidade Federal de Minas Gerais - UFMG

This is an API to read data from a DHT11 sensor with a STM32fxxx family microncontroller developed with System Workbench + STM32CubeMX.  For more information about the DHT11 sensor, please check the datasheet provided in the repository. Boards used in development: 
  - STM32F103C8Tx board, "Blue Pill" : https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html;
  - STM32F407VET6 board, "Black Board" : https://stm32-base.org/boards/STM32F407VET6-STM32-F4VE-V2.0.html.

This API was developed as a work of the Embedded Systems Programming discipline from UFMG - Prof. Ricardo de Oliveira Duarte - Department of Electronic Engineering.

Version 1.0 - API with the following implemented functions:
- void DHT11_init(GPIO_TypeDef* port, uint16_t pin);
- uint8_t DHT11_rawread();
- uint8_t DHT11_rh();
- float DHT11_temp();
- float DHT11_dewpoint();
- float DHT11_ah();

## Setup
To setup and use the API properly you must accomplish the following steps:
  - Set your clock tree and TIM1 (or other timer) to generate an interruption for each us (1MHz frequency);
  - Copy dht11.h and dht11.c to Inc and Src folders respectively;
  - In Inc/dht11.h, include the properly HAL library according to your microcontroller family;
  - A DHT_struct variable must be declared in your main function, named Dstruct;
  - The DHT11_init() function should be called once in your setup;

## Recommendations
The API will return 0xFF (255 in decimal) in temperature/humidity values if reading of the sensor was not done correctly. If that is the case, you may try:
- Check if your timer was configured correcly. Keep in mind that higher frequency timers showed better results in the development of this API;  
- New readings should not be done in intervals less than 1 second, as specified in the DHT11 Datasheet. In that case, the API design garantees that the Dstruct variable will remain with values of the last call;
- Check your power supply. The DHT11 sensor works properly with a 5V power supply (and your STM32fxxx probably works with 3v3), so ensure there is a voltage divisor between DHT11 Data pin and your MCU I/O used to read it.
- Some functions of the API return float values, so ensure your microcontroller has a Floating Point Unit;

## Demo Instructions
You will find attached two API demos in this repository, one for the stm32f103c8t6 microcontroller and another for the stm32f407vet6 microcontroller.
They are structered around a Finite State Machine that changes between the API functions every 3500 milliseconds, showing function results in a 128x64 OLED Display (Display Model: https://www.eletrogate.com/display-oled-0-96-i2c-azul, Library published by Alexander Lutsai, found in: https://github.com/SL-RU/stm32libs). The display uses a I2C interface, which requires two pin connections:
  - Pin PB6 : I2C_SCL, Pin PB7 : I2C_SDA for the stm32f103c8tx-dht11-api-demo;
  - Pin PB8 : I2C_SCL, Pin PB7 : I2C_SDA for the stm32f407VETx-dht11-api-demo.

The DHT11 sensor is connected to the PA1 pin in both boards.

Connection Diagram for the BluePill Demo:
![BluePillDemoDiagram](https://user-images.githubusercontent.com/71671310/108790976-c48dcb00-755c-11eb-9d66-b2f78e1b44bb.jpg)

## Licence
This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
For more information, read the LICENCE file available in the project root.

## References
- https://circuitdigest.com/microcontroller-projects/interfacing-dht11-sensor-with-stm32f103c8
- https://controllerstech.com/using-dht11-sensor-with-stm32/
- https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
- https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html
- https://stm32-base.org/boards/STM32F407VET6-STM32-F4VE-V2.0.html
- https://github.com/SL-RU/stm32libs
- Slides provided by UFMG - Prof. Ricardo de Oliveira Duarte - Department of Electronic Engineering
- DHT11 Datasheet available in the project root

