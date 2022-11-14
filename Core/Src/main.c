/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CAN.h"
#include "UART_LIN.h"
#include "myTime.h"
#include "LIN-Operator.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#include "UART_LIN.h"
LINMSG Tx_Msg;
LINMSG Rx_Msg;
Time RTCget;

CAN_RxHeaderTypeDef   RxHeader;
uint8_t RxData[8];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CAN_HandleTypeDef hcan1;


osThreadId UpdateExtClockHandle;
osThreadId UpdateOwnClockHandle;
osThreadId LIN_SelectorHandle;
osThreadId Op_RTCtoLINHandle;
osThreadId Op_LINtoRTCHandle;
osThreadId Op_LINtoCANHandle;
osThreadId Op_LEDtoLINHandle;
osThreadId Op_CANtoLINHandle;
osThreadId IDLEHandle;
osThreadId Op_LINtoLEDHandle;
osMessageQId Queue_CAN_InterruptHandle;
osMessageQId Queue_LIN_TO_RTCHandle;
osTimerId TimerClockHandle;
/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;


//Supprimer ici les 3 fonctions d'initialisation CAN, USART et RTCC
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void CallUpdateExtClock(void const * argument);
void CallUpdateOwnClock(void const * argument);
void StartTask_LIN_Selector(void const * argument);
void StartOp_RTCtoLIN(void const * argument);
void StartOp_LINtoRTC(void const * argument);
void StartTaskOp_LINtoCAN(void const * argument);
void StartTaskOp_LEDtoLIN(void const * argument);
void StartTask_Op_CANtoLIN(void const * argument);
void StartTaskIDLE(void const * argument);
void StartTask_Op_LINtoLED(void const * argument);
void CBClock(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	//SendLINRequest(&Tx_Msg);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  //Supprimer MX_USART3_Init
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  clock_Init();
  UART_Init();
  HAL_NVIC_DisableIRQ(USART3_IRQn);

  //MX_CAN1_Init();

  Init_PortB();
  lcd_init();
  lcd_puts(1, "Initialisation");

  HAL_CAN_Start(&hcan1);

  LED_Blink(3);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of TimerClock */
  osTimerDef(TimerClock, CBClock);
  TimerClockHandle = osTimerCreate(osTimer(TimerClock), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones,   .. */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue_CAN_Interrupt */
  osMessageQDef(Queue_CAN_Interrupt, 128, uint8_t);
  Queue_CAN_InterruptHandle = osMessageCreate(osMessageQ(Queue_CAN_Interrupt), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQDef(Queue_LIN_TO_RTC, 128, sizeof( Time ));
  Queue_LIN_TO_RTCHandle = osMessageCreate(osMessageQ(Queue_LIN_TO_RTC), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UpdateExtClock */
  osThreadDef(UpdateExtClock, CallUpdateExtClock, osPriorityNormal , 0, 128);
  UpdateExtClockHandle = osThreadCreate(osThread(UpdateExtClock), NULL);

  /* definition and creation of UpdateOwnClock */
  osThreadDef(UpdateOwnClock, CallUpdateOwnClock, osPriorityNormal , 0, 128);
  UpdateOwnClockHandle = osThreadCreate(osThread(UpdateOwnClock), NULL);

  /* definition and creation of LIN_Selector */
  osThreadDef(LIN_Selector, StartTask_LIN_Selector, osPriorityNormal, 0, 128);
  LIN_SelectorHandle = osThreadCreate(osThread(LIN_Selector), NULL);

  /* definition and creation of Op_RTCtoLIN */
  osThreadDef(Op_RTCtoLIN, StartOp_RTCtoLIN, osPriorityNormal, 0, 128);
  Op_RTCtoLINHandle = osThreadCreate(osThread(Op_RTCtoLIN), NULL);

  /* definition and creation of Op_LINtoRTC */
  osThreadDef(Op_LINtoRTC, StartOp_LINtoRTC, osPriorityNormal, 0, 128);
  Op_LINtoRTCHandle = osThreadCreate(osThread(Op_LINtoRTC), NULL);

  /* definition and creation of Op_LINtoCAN */
  osThreadDef(Op_LINtoCAN, StartTaskOp_LINtoCAN, osPriorityNormal, 0, 128);
  Op_LINtoCANHandle = osThreadCreate(osThread(Op_LINtoCAN), NULL);

  /* definition and creation of Op_LEDtoLIN */
  osThreadDef(Op_LEDtoLIN, StartTaskOp_LEDtoLIN, osPriorityNormal, 0, 128);
  Op_LEDtoLINHandle = osThreadCreate(osThread(Op_LEDtoLIN), NULL);

  /* definition and creation of Op_CANtoLIN */
  osThreadDef(Op_CANtoLIN, StartTask_Op_CANtoLIN, osPriorityNormal, 0, 128);
  Op_CANtoLINHandle = osThreadCreate(osThread(Op_CANtoLIN), NULL);

  /* definition and creation of IDLE */
  osThreadDef(IDLE, StartTaskIDLE, osPriorityIdle, 0, 128);
  IDLEHandle = osThreadCreate(osThread(IDLE), NULL);

  /* definition and creation of Op_LINtoLED */
  osThreadDef(Op_LINtoLED, StartTask_Op_LINtoLED, osPriorityLow, 0, 128);
  Op_LINtoLEDHandle = osThreadCreate(osThread(Op_LINtoLED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */

  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1){
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_CallUpdateExtClock */
/**
  * @brief  Function implementing the UpdateExtClock thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CallUpdateExtClock */
void CallUpdateExtClock(void const * argument)	//Envoi requete RTC par LIN
{
	/* USER CODE BEGIN 5 */

	HAL_NVIC_EnableIRQ(USART3_IRQn);
	/* Infinite loop */
	for(;;)
	{
		Tx_Msg.length = 1;
		Tx_Msg.ID = 0x90;
		SendLINRequest(&Tx_Msg);
		osDelay(800);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CallUpdateOwnClock */
/**
* @brief Function implementing the UpdateOwnClock thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CallUpdateOwnClock */
void CallUpdateOwnClock(void const * argument)	//RTC to LCD
{
  /* USER CODE BEGIN CallUpdateOwnClock */
	Time Data;
  /* Infinite loop */
  for(;;)
  {
    getCurrentTime(&Data);
    char chaine[17] = "                ";
    sprintf(chaine, "RTC : %02d:%02d:%02d" , Data.hou.BIN, Data.min.BIN, Data.sec.BIN);
    lcd_puts(1, chaine);
    osDelay(800);
  }
  /* USER CODE END CallUpdateOwnClock */
}

/* USER CODE BEGIN Header_StartTask_LIN_Selector */
/**
* @brief Function implementing the LIN_Selector thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_LIN_Selector */
void StartTask_LIN_Selector(void const * argument)
{
	/* USER CODE BEGIN StartTask_LIN_Selector */
	/* Infinite loop */
	for(;;)
	{
		osSignalWait(SIGNAL_LIN_INTERRUPT,osWaitForever);
		Rx_Msg.ID >>= 4;
		char mode = Rx_Msg.ID>>3;
		//Sinon ID = ID >> 4;

		switch (Rx_Msg.ID & 0b111){
		case LIN_ID_RTC :
			//RTC
			if (mode){
				osSignalSet(Op_RTCtoLINHandle,SIGNAL_RTC_TO_LIN);	 //RTC to LIN Task
			}
			else{
				Time LINTime;
				LINTime.hou.BIN = Tx_Msg.data[0];
				LINTime.min.BIN = Tx_Msg.data[1];
				LINTime.sec.BIN = Tx_Msg.data[2];
				xQueueSend (Queue_LIN_TO_RTCHandle , (void *) &LINTime , 100);	 //LIN to RTC Task
			}
			break;
		case LIN_ID_LED :
			//LED
			if (mode){
				osSignalSet(Op_LEDtoLINHandle,SIGNAL_LED_TO_LIN);	 //LED to LIN Task
			}
			else{
				osSignalSet(Op_LINtoLEDHandle,SIGNAL_LIN_TO_LED);	 //RTC to LIN Task
			}
			break;
		case LIN_ID_CAN :
			//CAN
			osSignalSet(Op_LINtoCANHandle,SIGNAL_LIN_TO_CAN);	 //RTC to LIN Task
			break;
		}
	}
	/* USER CODE END StartTask_LIN_Selector */
}

/* USER CODE BEGIN Header_StartOp_RTCtoLIN */
/**
* @brief Function implementing the Op_RTCtoLIN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOp_RTCtoLIN */
void StartOp_RTCtoLIN(void const * argument)
{
  /* USER CODE BEGIN StartOp_RTCtoLIN */
  /* Infinite loop */
  for(;;)
  {
	  osSignalWait(SIGNAL_RTC_TO_LIN,osWaitForever);

	  getCurrentTime(&RTCget);

	  Tx_Msg.data[0] = RTCget.hou.BIN;
	  Tx_Msg.data[1] = RTCget.min.BIN;
	  Tx_Msg.data[2] = RTCget.sec.BIN;
	  Tx_Msg.length = 3;
	  Tx_Msg.ID = LIN_ID(LIN_MODE_DATA, LIN_ID_RTC, Tx_Msg.length);
	  SendLINMessage(&Tx_Msg);
	  //Pas d'affichage LCD
  }
  /* USER CODE END StartOp_RTCtoLIN */
}

/* USER CODE BEGIN Header_StartOp_LINtoRTC */
/**
* @brief Function implementing the Op_LINtoRTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOp_LINtoRTC */
void StartOp_LINtoRTC(void const * argument)
{
  /* USER CODE BEGIN StartOp_LINtoRTC */
	Time Data;
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
		xQueueReceive(Queue_LIN_TO_RTCHandle, &(Data), osWaitForever);
		//Recevra une donnée de type RTC Time
		char chaine[17] = "                ";
		sprintf(chaine, "Ext : %02d:%02d:%02d", Data.hou.BIN,Data.min.BIN,Data.sec.BIN);
		lcd_puts(2, chaine);
	}
  /* USER CODE END StartOp_LINtoRTC */
}

/* USER CODE BEGIN Header_StartTaskOp_LINtoCAN */
/**
* @brief Function implementing the Op_LINtoCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOp_LINtoCAN */
void StartTaskOp_LINtoCAN(void const * argument)
{
	/* USER CODE BEGIN StartTaskOp_LINtoCAN */
	/* Infinite loop */
	for(;;)
	{
		osSignalWait(SIGNAL_LIN_TO_CAN,osWaitForever);
		if(Rx_Msg.data[0] == 0b1){
			CAN_Act(0x52, PORT_A, PIN_0_ON);
		}
		else{ //Afin d'éviter toute erreur d'envoi
			CAN_Act(0x52, PORT_A, PIN_ALL_OFF);
		}
		lcd_puts(1, "CAN Operation");
		lcd_puts(2, "Received     ");
	}
	/* USER CODE END StartTaskOp_LINtoCAN */
}

/* USER CODE BEGIN Header_StartTaskOp_LEDtoLIN */
/**
* @brief Function implementing the Op_LEDtoLIN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOp_LEDtoLIN */
void StartTaskOp_LEDtoLIN(void const * argument)
{
  /* USER CODE BEGIN StartTaskOp_LEDtoLIN */
  /* Infinite loop */
  for(;;)
  {
	  osSignalWait(SIGNAL_LED_TO_LIN,osWaitForever);
	  Tx_Msg.data[0] = 	(GPIOD->ODR >> 12 > 0);
	  Tx_Msg.length = 1;
	  Tx_Msg.ID = LIN_ID(LIN_MODE_DATA, LIN_ID_LED, Tx_Msg.length);
	  SendLINMessage(&Tx_Msg);
	  //Pas d'affichage LCD
  }
  /* USER CODE END StartTaskOp_LEDtoLIN */
}

/* USER CODE BEGIN Header_StartTask_Op_CANtoLIN */
/**
* @brief Function implementing the Op_CANtoLIN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Op_CANtoLIN */
void StartTask_Op_CANtoLIN(void const * argument)
{
  /* USER CODE BEGIN StartTask_Op_CANtoLIN */
	uint8_t Data;
  /* Infinite loop */
  for(;;)
  {
	  osDelay(100);
	  xQueueReceive(Queue_CAN_InterruptHandle, &Data, osWaitForever);
	  //revalue=osMessageGet(Op_CANtoLINHandle,10000);
	  Tx_Msg.data[0] = (Data > 0);
	  Tx_Msg.length = 1;
	  Tx_Msg.ID = LIN_ID(LIN_MODE_DATA, LIN_ID_CAN, Tx_Msg.length);
	  	SendLINMessage(&Tx_Msg);

	  	lcd_puts(1, "CAN Operation");
	  	lcd_puts(2, "Transmitted  ");
  }
  /* USER CODE END StartTask_Op_CANtoLIN */
}

/* USER CODE BEGIN Header_StartTaskIDLE */
/**
* @brief Function implementing the IDLE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIDLE */
void StartTaskIDLE(void const * argument)
{
  /* USER CODE BEGIN StartTaskIDLE */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskIDLE */
}

/* USER CODE BEGIN Header_StartTask_Op_LINtoLED */
/**
* @brief Function implementing the Op_LINtoLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Op_LINtoLED */
void StartTask_Op_LINtoLED(void const * argument)
{
  /* USER CODE BEGIN StartTask_Op_LINtoLED */
  /* Infinite loop */
  for(;;)
  {
	  osSignalWait(SIGNAL_LIN_TO_LED,osWaitForever);
		LED_Operate(Rx_Msg.data[0]);
		//Pas d'affichage LCD
  }
  /* USER CODE END StartTask_Op_LINtoLED */
}

/* CBClock function */
void CBClock(void const * argument)
{
  /* USER CODE BEGIN CBClock */

  /* USER CODE END CBClock */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  LED_Blink(0);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
