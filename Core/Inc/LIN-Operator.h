/*
 * CANLIN-operator.h
 *
 *  Created on: Jun 7, 2022
 *      Author: basti
 */

#ifndef INC_LIN_OPERATOR_H_
#define INC_LIN_OPERATOR_H_

#include "myTime.h"

#define LIN_MODE_DATA 0
#define LIN_MODE_REQUEST 1


#define LIN_ID(MODE, ID, LEN) ((MODE <<3 | ID) << 4 | LEN)

//fonction de traîtement des arrivées LIN
void LIN_USART_IRQ_Selector(uint8_t ID, uint8_t Data[10]);

//Fonctions concernant la RTC
#define LIN_ID_RTC 1
void Op_RTCtoLIN(void);
void Op_LINtoRTC(Time Data);

//Fonctions concernant la LED
#define LIN_ID_LED 2
void Op_LEDtoLIN(void);
void Op_LINtoLED(uint8_t Data);

//Fonctions qui seront appelées sur interruptions CAN ou LIN, concernant le CAN
#define LIN_ID_CAN 4
void Op_CANtoLIN(uint8_t Data);
void Op_LINtoCAN(uint8_t Data);

#endif /* INC_LIN_OPERATOR_H_ */
