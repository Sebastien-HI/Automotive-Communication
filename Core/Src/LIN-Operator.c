/*
 * CANLIN-Operator.c
 *
 *  Created on: Jun 7, 2022
 *      Author: basti
 */

#include "UART_LIN.h"
#include "CAN.h"
#include "LIN-Operator.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

void LIN_USART_IRQ_Selector(uint8_t ID, uint8_t Data[10]){
	ID >>= 4;
	char mode = ID>>3;
	//Sinon ID = ID >> 4;

	switch (ID & 0b111){
	case LIN_ID_RTC :
		//RTC
		if (mode){
			Op_RTCtoLIN();
		}
		else{
			Time LINTime;
			LINTime.hou.BIN = Data[0];
			LINTime.min.BIN = Data[1];
			LINTime.sec.BIN = Data[2];
			Op_LINtoRTC(LINTime);
		}
		break;
	case LIN_ID_LED :
		//LED
		if (mode){
			Op_LEDtoLIN();
		}
		else{
			Op_LINtoLED(Data[0]);
		}
		break;
	case LIN_ID_CAN :
		//CAN
		Op_LINtoCAN(Data[0]);
		break;
	}
}

void Op_CANtoLIN(uint8_t Data){
	LINMSG Operator;
	Operator.data[0] = Data;
	Operator.length = 1;
	Operator.ID = LIN_ID(LIN_MODE_DATA, LIN_ID_CAN, Operator.length);
	SendLINMessage(&Operator);

	lcd_puts(1, "CAN Operation");
	lcd_puts(2, "Transmitted  ");
}

void Op_LINtoCAN(uint8_t Data){
	if(Data == 0b1){
		CAN_Act(0x52, PORT_A, PIN_0_ON);
	}
	else{ //Afin d'éviter toute erreur d'envoi
		CAN_Act(0x52, PORT_A, PIN_ALL_OFF);
	}
	lcd_puts(1, "CAN Operation");
	lcd_puts(2, "Received     ");
}

void Op_RTCtoLIN(void){
	Time RTCget;
	getCurrentTime(&RTCget);

	LINMSG Operator;
	Operator.data[0] = RTCget.hou.BIN;
	Operator.data[1] = RTCget.min.BIN;
	Operator.data[2] = RTCget.sec.BIN;
	Operator.length = 3;
	Operator.ID = LIN_ID(LIN_MODE_DATA, LIN_ID_RTC, Operator.length);
	SendLINMessage(&Operator);
	//Pas d'affichage LCD
}

void Op_LINtoRTC(Time Data){
	//Recevra une donnée de type RTC Time
	char chaine[20] = "";
	sprintf("Ext : %d:%d %d", chaine, Data.hou.BIN,Data.min.BIN,Data.sec.BIN);
	lcd_puts(2, chaine);
}

void Op_LEDtoLIN(void){
	LINMSG Operator;
	Operator.data[0] = 	(GPIOD->ODR >> 12 > 0);
	Operator.length = 1;
	Operator.ID = LIN_ID(LIN_MODE_DATA, LIN_ID_LED, Operator.length);
	SendLINMessage(&Operator);
	//Pas d'affichage LCD
}

void Op_LINtoLED(uint8_t Data){
	//LED_Operate(Rx_Msg.Data[0]);
	//Pas d'affichage LCD
}


