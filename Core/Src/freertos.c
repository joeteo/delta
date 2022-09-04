/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dxl_control.h"
#include "DeltaKinematics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
	float mX;
	float mY;
	float mZ;
	int maxSpeed;
	uint8_t timing;

}queueMessage;

extern uint16_t GP[3];
extern double coord[3];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart3;

extern _Coordinates C;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId calWritePosTaskHandle;
osMessageQId setQueueHandle;
osSemaphoreId ReadPosSemHandle;
osSemaphoreId TorqueOnSemHandle;
osSemaphoreId TorqueOffSemHandle;
osSemaphoreId PumpOnSemHandle;
osSemaphoreId PumpOffSemHandle;
osSemaphoreId ConveyorRightOnSemHandle;
osSemaphoreId ConveyorLeftOnSemHandle;
osSemaphoreId ConveyorOffSemHandle;
osSemaphoreId UPSemHandle;
osSemaphoreId DOWMSemHandle;
osSemaphoreId ThrowSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void cal_Write_Pos_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	deltaInit();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ReadPosSem */
  osSemaphoreDef(ReadPosSem);
  ReadPosSemHandle = osSemaphoreCreate(osSemaphore(ReadPosSem), 1);

  /* definition and creation of TorqueOnSem */
  osSemaphoreDef(TorqueOnSem);
  TorqueOnSemHandle = osSemaphoreCreate(osSemaphore(TorqueOnSem), 1);

  /* definition and creation of TorqueOffSem */
  osSemaphoreDef(TorqueOffSem);
  TorqueOffSemHandle = osSemaphoreCreate(osSemaphore(TorqueOffSem), 1);

  /* definition and creation of PumpOnSem */
  osSemaphoreDef(PumpOnSem);
  PumpOnSemHandle = osSemaphoreCreate(osSemaphore(PumpOnSem), 1);

  /* definition and creation of PumpOffSem */
  osSemaphoreDef(PumpOffSem);
  PumpOffSemHandle = osSemaphoreCreate(osSemaphore(PumpOffSem), 1);

  /* definition and creation of ConveyorRightOnSem */
  osSemaphoreDef(ConveyorRightOnSem);
  ConveyorRightOnSemHandle = osSemaphoreCreate(osSemaphore(ConveyorRightOnSem), 1);

  /* definition and creation of ConveyorLeftOnSem */
  osSemaphoreDef(ConveyorLeftOnSem);
  ConveyorLeftOnSemHandle = osSemaphoreCreate(osSemaphore(ConveyorLeftOnSem), 1);

  /* definition and creation of ConveyorOffSem */
  osSemaphoreDef(ConveyorOffSem);
  ConveyorOffSemHandle = osSemaphoreCreate(osSemaphore(ConveyorOffSem), 1);

  /* definition and creation of UPSem */
  osSemaphoreDef(UPSem);
  UPSemHandle = osSemaphoreCreate(osSemaphore(UPSem), 10);

  /* definition and creation of DOWMSem */
  osSemaphoreDef(DOWMSem);
  DOWMSemHandle = osSemaphoreCreate(osSemaphore(DOWMSem), 10);

  /* definition and creation of ThrowSem */
  osSemaphoreDef(ThrowSem);
  ThrowSemHandle = osSemaphoreCreate(osSemaphore(ThrowSem), 10);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

	for(int i=0; i<10; i++){
		osSemaphoreWait(UPSemHandle, 0);
		osSemaphoreWait(DOWMSemHandle, 0);
		osSemaphoreWait(ThrowSemHandle, 0);
	}
	osSemaphoreWait(ReadPosSemHandle, 0);
	osSemaphoreWait(TorqueOnSemHandle, 0);
	osSemaphoreWait(TorqueOffSemHandle, 0);


  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of setQueue */
  osMessageQDef(setQueue, 10, queueMessage);
  setQueueHandle = osMessageCreate(osMessageQ(setQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of calWritePosTask */
  osThreadDef(calWritePosTask, cal_Write_Pos_Task, osPriorityNormal, 0, 512);
  calWritePosTaskHandle = osThreadCreate(osThread(calWritePosTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

	  if(UPSemHandle != NULL)
	  {
		  if(osSemaphoreWait(UPSemHandle, 0) == osOK)
		  {

			  queueMessage smsg;
			  smsg.mX=0;
			  smsg.mY=0;
			  smsg.mZ=-256.984;
			  smsg.maxSpeed=100;
			  smsg.timing=0;


			  osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);


//			  upEndEffector();
		  }
	  }

	  if(DOWMSemHandle != NULL)
	  {
		  if(osSemaphoreWait(DOWMSemHandle, 0) == osOK)
		  {


			  queueMessage smsg;
			  smsg.mX=0;
			  smsg.mY=0;
			  //smsg.mZ=-380.724;
			  smsg.mZ=-407.891;

			  smsg.maxSpeed=100;
			  smsg.timing=1;

			  osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);




//			  downEndEffector();
		  }
	  }

	  if(ReadPosSemHandle != NULL)
	  {
		  if(osSemaphoreWait(ReadPosSemHandle, 0) == osOK)
		  {
			  osThreadSetPriority(defaultTaskHandle, osPriorityAboveNormal);
			  uint8_t buf[30];
			  uint16_t presentPos[3];
			  for(int i = 0; i < 3; i++){
				  memset(buf,0,sizeof(buf));
				  presentPos[i]=getPresentPosition(i);
				  sprintf((char*)buf, "ID %d's Position : %d\r\n", i, presentPos[i]);
				  HAL_UART_Transmit(&huart3, buf, sizeof(buf), 1000);
			  }
			  double* tempTheta = ConversionFromServo(presentPos[0], presentPos[1], presentPos[2]);
			  forward(tempTheta[0],tempTheta[1],tempTheta[2]);
			  memset(buf,0,sizeof(buf));
			  sprintf((char*)buf, "Coordinate X : %lf\r\n", coord[0]);
			  HAL_UART_Transmit(&huart3, buf, sizeof(buf), 1000);
			  memset(buf,0,sizeof(buf));
			  sprintf((char*)buf, "Coordinate Y : %lf\r\n", coord[1]);
			  HAL_UART_Transmit(&huart3, buf, sizeof(buf), 1000);
			  memset(buf,0,sizeof(buf));
			  sprintf((char*)buf, "Coordinate Z : %lf\r\n", coord[2]);
			  HAL_UART_Transmit(&huart3, buf, sizeof(buf), 1000);


			  osThreadSetPriority(defaultTaskHandle, osPriorityNormal);
		  }
	  }

	  if(TorqueOffSemHandle != NULL)
	  {
		  if(osSemaphoreWait(TorqueOffSemHandle, 0) == osOK)
		  {
			  //onOffTorque(AX_BROADCAST_ID, OFF);

			  syncWriteTorqueOnOff(OFF);
		  }
	  }

	  if(TorqueOnSemHandle != NULL)
	  {
		  if(osSemaphoreWait(TorqueOnSemHandle, 0) == osOK)
		  {
			  //onOffTorque(AX_BROADCAST_ID, ON);

			  syncWriteTorqueOnOff(ON);

//			  queueMessage smsg;
//			  smsg.mX=0;
//			  smsg.mY=90;
//			  smsg.mZ=-370;
//			  smsg.maxSpeed=50;
//			  smsg.timing=0;
//
//			  osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);
		  }
	  }

	  if(ThrowSemHandle != NULL)
	  {
		  if(osSemaphoreWait(ThrowSemHandle, 0) == osOK)
		  {

			  queueMessage smsg;
			  smsg.mX=0;
			  smsg.mY=-140;
			  smsg.mZ=-230;
			  smsg.maxSpeed=1000;
			  smsg.timing=2;

			  osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);

		  }
	  }

	  if(PumpOnSemHandle != NULL)
	  {
		  if(osSemaphoreWait(PumpOnSemHandle, 0) == osOK)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
		  }
	  }
	  if(PumpOffSemHandle != NULL)
	  {
		  if(osSemaphoreWait(PumpOffSemHandle, 0) == osOK)
		  {
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
		  }
	  }


	  if(ConveyorRightOnSemHandle != NULL)
	  {
		  if(osSemaphoreWait(ConveyorRightOnSemHandle, 0) == osOK)
		  {
			  setEndless(AX_CONVEYOR_ID, ON);
			  turn(AX_CONVEYOR_ID, RIGHT, 600);
		  }
	  }
	  if(ConveyorLeftOnSemHandle != NULL)
	  {
		  if(osSemaphoreWait(ConveyorLeftOnSemHandle, 0) == osOK)
		  {
			  setEndless(AX_CONVEYOR_ID, ON);
			  turn(AX_CONVEYOR_ID, LEFT, 600);
		  }
	  }
	  if(ConveyorOffSemHandle != NULL)
	  {
		  if(osSemaphoreWait(ConveyorOffSemHandle, 0) == osOK)
		  {
			  onOffTorque(AX_CONVEYOR_ID, OFF);
			  //setEndless(AX_CONVEYOR_ID, OFF);
		  }
	  }


  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_cal_Write_Pos_Task */
/**
* @brief Function implementing the calWritePosTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cal_Write_Pos_Task */
void cal_Write_Pos_Task(void const * argument)
{
  /* USER CODE BEGIN cal_Write_Pos_Task */
  /* Infinite loop */
  for(;;)
  {
	  osEvent setEvent;
	  setEvent = osMessageGet(setQueueHandle, osWaitForever);
		if(setEvent.status == osEventMessage)
		{
			queueMessage *msgp;
			msgp = setEvent.value.p;
			queueMessage msg = *(msgp);

			printf("X : %lf \r\n",msg.mX);
			printf("Y : %lf \r\n",msg.mY);
			printf("Z : %lf \r\n",msg.mZ);
			printf("Speed : %d \r\n",msg.maxSpeed);

			setCoordinates((double)msg.mX,(double)msg.mY,(double)msg.mZ);

			uint16_t GPBefore[3];
			memcpy(GPBefore, GP, sizeof(uint16_t)*3);

			inverse();
			ServoConversion();
			uint16_t diff[3];

			diff[0]=abs(GP[0]-GPBefore[0]);
			diff[1]=abs(GP[1]-GPBefore[1]);
			diff[2]=abs(GP[2]-GPBefore[2]);

			uint16_t max = (diff[0] > diff[1] && diff[0] > diff[2]) ? diff[0] : (diff[1] > diff[2]) ? diff[1] : diff[2];


			double speed[3]={0,};
			speed[0]=((double)diff[0]/max)*msg.maxSpeed;
			speed[1]=((double)diff[1]/max)*msg.maxSpeed;
			speed[2]=((double)diff[2]/max)*msg.maxSpeed;


			syncWriteGoalPosition(GP[0],(uint16_t)speed[0],GP[1],(uint16_t)speed[1],GP[2],(uint16_t)speed[2]);
			servoDelay(10);
			if(msg.timing==2){
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
			}
			servoDelay(990);
			if(msg.timing==1){
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
			}


		}


  }
  /* USER CODE END cal_Write_Pos_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
