/* 
 * File:   UART_LIN.c
 * Author: aravey
 *
 * Created on 12 april 2020, 13:42
 */

#include "UART_LIN.h"
#include "main.h"
#include "LIN-Operator.h"
#include "string.h"
#include "cmsis_os.h"
extern LINMSG Tx_Msg;
extern LINMSG Rx_Msg;

UART_HandleTypeDef huart3;
//Functions
extern osThreadId LIN_SelectorHandle;

//UART_Init() sets up the UART for a 8-bit data, No Parity, 1 Stop bit
//at 9600 baud with transmitter interrupts enabled
void UART_Init (void)
{
	/* Enable interrupt */
	NVIC_EnableIRQ(USART3_IRQn);

	//enable GPIO
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

  
    //enable USART3
	  huart3.Instance = USART3;
	  huart3.Init.BaudRate = 9600;
	  huart3.Init.WordLength = UART_WORDLENGTH_8B;
	  huart3.Init.StopBits = UART_STOPBITS_1;
	  huart3.Init.Parity = UART_PARITY_NONE;
	  huart3.Init.Mode = UART_MODE_TX_RX;
	  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_LIN_Init(&huart3, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
	  {
	    Error_Handler();
	  }
		uint32_t prioritygroup = 0x00U;
		prioritygroup = NVIC_GetPriorityGrouping();
		NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(prioritygroup, 15, 15));

	  __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
  
    //USART3->CR2 |= 0x00004020;	//enable LIN mode and LBDL a desactiver pour dernier test final interruption com
    USART3->CR2 |= 0x00004060;	//enable LIN mode, LBDIE and LBDL for break detection
}

void USART3_IRQHandler(void)
{
	uint8_t calcul;
	uint8_t checksum1;
	uint8_t trash;
	uint8_t trash2;

	HAL_UART_IRQHandler(&huart3);
	if (USART3->SR & USART_SR_LBD_Msk) //Si une trame arrive
	{
		USART3->SR &= ~(USART_SR_LBD_Msk);//RAZ flag
		timeout();
		trash = USART3->DR;	//lecture pour remise à 0 du registre
		timeout();
		trash2 = USART3->DR;//lecture pour remise à 0 du registre
		timeout();

		Rx_Msg.ID = USART3->DR;
		if( Rx_Msg.ID | 0xF > 0 )
		{
			//mode reception message
			timeout();
			Rx_Msg.length = Rx_Msg.ID & 0xF;	//recupération du nombre des data
			for(int i = 0; i<Rx_Msg.length-1;i++)
			{
				Rx_Msg.data[i] = USART3->DR;
				timeout();
			}
			checksum1 = USART3->DR;
			calcul=checksum(Rx_Msg.length, Rx_Msg.data);

			//Si le checksum est mauvais, on flush toutes les données
			if(checksum1 != calcul)
			{
				for(int i=0;i<10;i++)
				{
					Rx_Msg.data[i] = 0;
				}
			}
		}
					
		osSignalSet(LIN_SelectorHandle,SIGNAL_LIN_INTERRUPT);

		//fin de l'interruption
	}
}
	
void timeout(void)
{
	volatile int compteur=0;
	int timeout=102000;
	while(!(USART3->SR & 0x00000020) && timeout>compteur) //RXNE donnée prete a être lue
	{
		compteur++;
	}
	compteur=0;
}	


/*--- Transmit LIN Message ---*/

	void SendLINMessage(LINMSG *data)
	  {
		 uint8_t calcul;
		 for(int i = (Tx_Msg.length-1); i>-1; i--)
		 {
			 UART_PutChar(Tx_Msg.data[i]);
		 }
		 calcul=checksum(Tx_Msg.length, Tx_Msg.data);
		 UART_PutChar(calcul);
	  }

/*--- Transmit LIN Request ---*/

void SendLINRequest(LINMSG *msg)
  {
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	sync_break();
	 //ID
	UART_PutChar(Tx_Msg.ID);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	if(Tx_Msg.ID & 0xF > 0) //si adresse > 0x24 alors il s'agit d'envoi maitre data
	{
		SendLINMessage(&Tx_Msg);
	}
  }


/*--- Send sync field +Tdel and break ---*/

void sync_break(void)
  {
	USART3->CR1 |= 0x00000001;   	//send Breaks
	UART_PutChar(0x55);				//send Syncs
  }

/*--- Transmit char ---*/
 
void UART_PutChar(uint8_t data)
  {
		//while(!(USART3->SR & 0x00000020)); 	//attends que registre envoi/reception libre
		USART3->DR = data;				//envoi la donnée
		while(!(USART3->SR & 0x00000080));	// donnee transferee au registre de decalage
		while(!(USART3->SR & 0x00000040));	//fin de transmission
  }

int slave_response(void)
{
	SendLINMessage(&Tx_Msg);
	return 0;
}

/*--- Calculate lin checksum ---*/

uint8_t checksum(uint8_t length, uint8_t *data)
  {
  uint8_t ix;
  uint16_t check_sum = 0;

  for(ix = 0; ix < length-1; ix++)
    {
    check_sum += data[ix];
    if(check_sum >= 256){
      check_sum -= 255;
      }
    }

  return (uint8_t)(0xff - check_sum);
  }
