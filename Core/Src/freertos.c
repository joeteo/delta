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
#include "cmdHandle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
osThreadId cmdHandleTaskHandle;
osMessageQId setQueueHandle;
osMessageQId cmdQueueHandle;
osSemaphoreId ReadPosSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void cal_Write_Pos_Task(void const * argument);
void cmd_Handle_Task(void const * argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */


	osSemaphoreWait(ReadPosSemHandle, 0);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of setQueue */
  osMessageQDef(setQueue, 10, queueMessage);
  setQueueHandle = osMessageCreate(osMessageQ(setQueue), NULL);

  /* definition and creation of cmdQueue */
  osMessageQDef(cmdQueue, 10, 20);
  cmdQueueHandle = osMessageCreate(osMessageQ(cmdQueue), NULL);

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

  /* definition and creation of cmdHandleTask */
  osThreadDef(cmdHandleTask, cmd_Handle_Task, osPriorityNormal, 0, 512);
  cmdHandleTaskHandle = osThreadCreate(osThread(cmdHandleTask), NULL);




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


	  if(osSemaphoreWait(ReadPosSemHandle, osWaitForever) == osOK)
	  {

		  //uint8_t buf[30];
		  uint16_t presentPos[3];
		  for(int i = 0; i < 3; i++){

			  presentPos[i]=getPresentPosition(i);

		  }
		  double* tempTheta = ConversionFromServo(presentPos[0], presentPos[1], presentPos[2]);
		  forward(tempTheta[0],tempTheta[1],tempTheta[2]);

		  char buf[14]="Z\0";
		  for(int i=0; i<3; i++){
			  if((int)coord[i] >= 0){
				  strcat(buf, "+");
			  }else{
				  strcat(buf, "-");
			  }
			  char temp[4]="\0";
			  sprintf(temp, "%03d",(int)abs(coord[i]));
			  strcat(buf, temp);
		  }
		  strcat(buf, "\n");

		  HAL_UART_Transmit(&huart3, (uint8_t*)buf, sizeof(buf), 1000);

		  osThreadSetPriority(defaultTaskHandle, osPriorityNormal);
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

			double speed[3]={100,100,100};

			if(max!=0){
				speed[0]=((double)diff[0]/max)*msg.maxSpeed;
				speed[1]=((double)diff[1]/max)*msg.maxSpeed;
				speed[2]=((double)diff[2]/max)*msg.maxSpeed;
				for(int i=0; i<3; i++){
					if(speed[i]<1) speed[i]=10;
				}
			}

			syncWriteGoalPosition(GP[0],(uint16_t)speed[0],GP[1],(uint16_t)speed[1],GP[2],(uint16_t)speed[2]);
			servoDelay(10);



		}


  }
  /* USER CODE END cal_Write_Pos_Task */
}

/* USER CODE BEGIN Header_cmd_Handle_Task */
/**
* @brief Function implementing the cmdHandleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cmd_Handle_Task */
void cmd_Handle_Task(void const * argument)
{
  /* USER CODE BEGIN cmd_Handle_Task */
  /* Infinite loop */
  for(;;)
  {
	  osEvent setEvent;
	  setEvent = osMessageGet(cmdQueueHandle, osWaitForever);
		if(setEvent.status == osEventMessage)
		{

			queueMessage msg;
			char cmd[20]={0,};
			memcpy(cmd, setEvent.value.p, 20);
			if(cmd_handler(cmd, &msg)){
				osSemaphoreRelease(ReadPosSemHandle);
				osThreadSetPriority(defaultTaskHandle, osPriorityAboveNormal);
			}

		}

  }
  /* USER CODE END cmd_Handle_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
