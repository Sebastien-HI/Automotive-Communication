/*
 * Ce fichier contient les fonctions diverses du projet
 * > Fonctions d'actions sur les LEDs
 * > Fonctions intermédiaires d'affichage
 * > Fonctions de gestion des contextes
 *
 * Projet SM57
 * BARNEOUD - HIRTH
 */

//Rotate les LEDs [loops] fois, 0 = infini

#include "Misc.h"

void temporisation(int temps){
  int temporisation = temps * 1000 * 21; //Approximation de la milliseconde
  while(temporisation -- > 0);
}

//Rotation des LEDs
void LED_Rotate(int loops){
	char ActualState = GPIOD->ODR;
	char inf = loops ? 1 : 0;
	do{
		for(int i = 0; i < 4; i++){
		  GPIOD->ODR = 0x00001000 << i;
		  temporisation(150);
		}
		loops --;
	}while((loops > 0) | !inf);
	GPIOD->ODR = ActualState;
}

//Blinke les LEDs [loops] fois, 0 = infini
void LED_Blink(int loops){
	char ActualState = GPIOD->ODR;
	char dec = loops ? 1 : 0;
	do{
		GPIOD->ODR = 0xF000;
		temporisation(150);
		GPIOD->ODR = 0x0000;
		temporisation(150);
		loops --;
	}while((loops > 0) |( dec == 0));

	GPIOD->ODR = ActualState;
}

//Allume toutes les LEDs
void LED_Operate(char State){
	GPIOD->ODR = 0xF000 * State;
}

