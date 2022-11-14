/* 
 * File:   UART_LIN.h
 * Author: aravey
 *
 * Created on 12 april 2020, 13:42
 */

#ifndef UART_LIN_H
#define	UART__LIN_H

#include <stdint.h>


//Declaration to Link External Functions & Variables:

/*--- Lin message structures and flag ---*/

typedef struct
  {
  uint8_t ID;
  uint8_t length;
  uint8_t data[10];
  }LINMSG;

//Variables with Global Scope  
extern  unsigned char DisplayData[];
extern  LINMSG Tx_Msg;
extern  LINMSG Rx_Msg;
extern int new_request;
//=0 mode maitre : attend reponse
//=1 mode esclave, => attente requete


//Functions :
void UART_Init (void);
void Error_Handler(void);
void USART3_IRQHandler(void);
void SendLINMessage(LINMSG *msg);
void SendLINRequest(LINMSG *msg);
void timeout(void);
int slave_response(void);
void UART_PutChar(uint8_t data);
void sync_break(void);
uint8_t checksum(uint8_t length, uint8_t *data);



#endif	/* UART_LIN_H */

