/*
 * uartRingBufDMA.c
 *
 *  Created on: Aug 12, 2021
 *      Author: controllerstech.com
 */


#include "stm32f4xx_hal.h"
#include "uartCallback.h"
#include "string.h"
#include <stdio.h>
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern UART_HandleTypeDef huart3;

extern osMessageQId cmdQueueHandle;


#define DMA hdma_usart2_rx

#define UART3 huart3


uint8_t rx3_data;
uint8_t rx3_buf[RxBuf_SIZE];

uint8_t rx_start = 0;
uint8_t bufindex = 0;

uint8_t rx2_Buf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];

uint16_t oldPos = 0;
uint16_t newPos = 0;


uint16_t Head, Tail;

/* Timeout is in milliseconds */
int32_t TIMEOUT = 0;



/* Initialize the Ring Buffer */
void UartCallback_Init (void)
{
	memset(rx2_Buf, '\0', RxBuf_SIZE);
	memset(MainBuf, '\0', MainBuf_SIZE);

	//Head = Tail = 0;
	oldPos = 0;
	newPos = 0;


  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_Buf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);


  HAL_UART_Receive_IT(&huart3, &rx3_data, 1);


}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		//isDataAvailable = 1;

	if(huart->Instance==USART2){
		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)MainBuf+oldPos, (uint8_t *)rx2_Buf, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf, (uint8_t *)rx2_Buf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)MainBuf+oldPos, (uint8_t *)rx2_Buf, Size);
			newPos = Size+oldPos;
		}

		/* Update the position of the Head
		 * If the current position + new size is less then the buffer size, Head will update normally
		 * Or else the head will be at the new position from the beginning
		 */
//		if (Head+Size < MainBuf_SIZE) Head = Head+Size;
//		else Head = Head+Size - MainBuf_SIZE;

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) rx2_Buf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART3){

		if(rx_start == 0){
			if(rx3_data == 'D'){
				bufindex = 0;
				rx_start = 1;
			}
		}
		else {
			if(rx3_data != '\n' && bufindex < RxBuf_SIZE)
				rx3_buf[bufindex++] = rx3_data;
			else {
				char temp_buf[20];
				memcpy(temp_buf, (char*)rx3_buf, 20);
				osMessagePut(cmdQueueHandle, (uint32_t)temp_buf, 100);
				memset(rx3_buf,0,sizeof(rx3_buf));
				bufindex=0;
				rx_start = 0;
			}
		}
		HAL_UART_Receive_IT(&huart3, &rx3_data, 1);

	}

}
