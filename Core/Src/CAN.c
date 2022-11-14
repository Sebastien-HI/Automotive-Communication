/**
 * Fichier de définition des fonctions liées au CAN STM
 * Projet SM57
 *
 * BARNEOUD - HIRTH
 */

#include "CAN.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               RxData[8];

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 17;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef Filtre_Reception;
  Filtre_Reception.FilterBank = 0;
  Filtre_Reception.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  Filtre_Reception.FilterActivation = CAN_FILTER_ENABLE;
  Filtre_Reception.FilterScale = CAN_FILTERSCALE_32BIT;
  Filtre_Reception.FilterMode = CAN_FILTERMODE_IDMASK;

  uint32_t IDref =  0x10005002 << 3 | 0b100; //IDs venant d'un esclave envoyant pour tout le réseau
  uint32_t Masque = 0x1F00FF00 << 3 | 0b000; //Valeurs qui filtreront les arrivages -> 0 parce que tout est canné

  Filtre_Reception.FilterIdHigh = IDref >> 16;
  Filtre_Reception.FilterIdLow = IDref & 0xFFFF;
  Filtre_Reception.FilterMaskIdHigh = Masque >> 16;
  Filtre_Reception.FilterMaskIdLow = Masque & 0xFFFF;

  HAL_CAN_ConfigFilter (&hcan1, &Filtre_Reception);

  //Exemple d'ID d'esclave à tout le réseau :  0x 1000 630F
  //Exemple d'ID de STM répondant à l'esclave :0x 1063 100F

  HAL_CAN_ActivateNotification (&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN1_Init 2 */

}

void CAN_Act(uint8_t Slave_Addr, uint8_t Port_Addr, int8_t Data){
	int32_t Adresse = 0x10000000 | (Slave_Addr << 16) | (CAN_MASTER_ADDR << 8) | Port_Addr; //Esclave + Port Read A
	CAN_Send(Adresse,Data);
}

/**
 * @brief CAN Send message
 * @retval none
 * Dans l'hypothèse qu'on aura pas besoin d'envoyer de trames avec plus d'un octet
 */
void CAN_Send(double Addr, int8_t Data){
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[8];
	uint32_t              TxMailbox;

	TxHeader.IDE = CAN_ID_EXT; //MODE CAN ETENDU
	TxHeader.ExtId = Addr;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 1;
	TxData[0] = Data;

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	 Error_Handler ();
	}
	//Message envoyé ?
	//while(HAL_CAN_IsTxMessagePending(&hcan1, &TxMailbox));

}

/**
 * @brief CAN get data from last received message
 * @retval received data
 */
uint8_t CAN_Get(void){
  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
	Error_Handler();
  }
  return RxData[0];
}
