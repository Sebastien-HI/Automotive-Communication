/*
 * Misc.h
 *
 *  Created on: 23 mai 2022
 *      Author: basti
 */

#ifndef SRC_MISC_H_
#define SRC_MISC_H_

#include "stm32f4xx_hal.h"

//Blinke les LEDs [loops] fois, 0 = infini
void LED_Blink(int loops);

//Affiche une animation de rotation des LEDs
void LED_Rotate(int loops);

//Opère les leds selon un état spontané
void LED_Operate(char State);

#endif /* SRC_MISC_H_ */
