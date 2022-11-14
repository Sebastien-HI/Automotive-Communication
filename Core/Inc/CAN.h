/*
 * CAN.h
 *
 *  Created on: 23 mai 2022
 *      Author: basti
 */

#ifndef SRC_CAN_H_
#define SRC_CAN_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define CAN_MASTER_ADDR 13 //Adresse CAN de notre carte : 0xD

//Initialisation du CAN
void MX_CAN1_Init(void);

//Envoyer un octet unique sur le CAN
void CAN_Send(double Addr, int8_t Data);

//Récupérer la donnée CAN
uint8_t CAN_Get(void);

void CAN_Act(uint8_t Slave_Addr, uint8_t Port_Addr, int8_t Data);


//VARIABLES

//Structure de résultat CAN
/*
#ifndef hcan1
	CAN_HandleTypeDef hcan1;
#endif
*/

#ifndef RxHeader
	extern CAN_RxHeaderTypeDef   RxHeader;
#endif
#ifndef RxData
	extern uint8_t               RxData[8];
#endif

//DEFINES
#define PORT_A 0x10
#define PORT_B 0x11
#define PORT_C 0x12
#define PORT_D 0x13


#define BIT_PIN(PIN) (1 << PIN)

#define PIN_ALL_OFF 0
#define PIN_0_ON BIT_PIN(0)
#define PIN_1_ON BIT_PIN(1)
#define PIN_2_ON BIT_PIN(2)
#define PIN_3_ON BIT_PIN(3)
#define PIN_4_ON BIT_PIN(4)
#define PIN_5_ON BIT_PIN(5)
#define PIN_6_ON BIT_PIN(6)
#define PIN_7_ON BIT_PIN(7)
#define PIN_8_ON BIT_PIN(8)


#endif /* SRC_CAN_H_ */
