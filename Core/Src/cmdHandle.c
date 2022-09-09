/*
 * cli.c
 *
 *  Created on: Sep 4, 2022
 *      Author: Jo soo hyun
 *
 *
 */

#include "cmdHandle.h"
#include "main.h"
#include "dxl_control.h"

#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart3;

#include "cmsis_os.h"


extern osMessageQId setQueueHandle;



int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart3, (uint8_t*)p, len, 10);
	return len;
}

struct Command_List  CmdList[] =
{
	{'T',	cmd_torque },
	{'P',	cdm_pump },
	{'C',	cmd_conveyorBelt },
	{'A',	cmd_pick },
	{'B',	cmd_throw },
	{'O',	cmd_defaultPos },
	{'R',	cmd_read },
	{'Z',	cmd_moveTo },
	{0,0 }
};



int cmd_torque(int len, char* cmd)
{

	if(len == 1) {
		if (*cmd=='1') {
			syncWriteTorqueOnOff(ON);
		}
		else if (*cmd=='0') {
			syncWriteTorqueOnOff(OFF);
		}
	}else {
		printf("wrong command pattern!");
	}

	return 0;
}

int cdm_pump(int len, char* cmd)
{
	if(len == 1) {
		if (*cmd=='1') {
			pumpOn();
		}
		else if (*cmd=='0') {
			pumpOff();
		}
	}else {
		printf("wrong command pattern!");
	}

	return 0;
}

int cmd_conveyorBelt(int len, char* cmd)
{
	if(len == 2) {
		if (*cmd=='1') {
			if (cmd[1]=='R') {
				cvbeltTurnRight();
			} else if (cmd[1]=='L'){
				cvbeltTurnLeft();
			}
		}
	}else if(len == 1){
		if (*cmd=='0') {
				cvbeltStop();
		}
	}else {
		printf("wrong command pattern!");
	}

	return 0;
}

int cmd_pick(int len, char* cmd)
{
	downEndEffector();
	pumpOn();
	servoDelay(1000);
	upEndEffector();
	return 0;
}

int cmd_throw(int len, char* cmd)
{
	throw();
	return 0;
}

int cmd_defaultPos(int len, char* cmd){
	upEndEffector();
	return 0;
}

int cmd_moveTo(int len, char* cmd){

	if(len == 12){

		float tempX = (cmd[1]-'0')*100 + (cmd[2]-'0')*10 + (cmd[3]-'0')*1 ;
		if(cmd[0]=='-'){
			tempX = -tempX;
		}
		float tempY = (cmd[5]-'0')*100 + (cmd[6]-'0')*10 + (cmd[7]-'0')*1 ;
		if(cmd[4]=='-'){
			tempY = -tempY;
		}
		float tempZ = (cmd[9]-'0')*100 + (cmd[10]-'0')*10 + (cmd[11]-'0')*1 ;
		if(cmd[8]=='-'){
			tempZ = -tempZ;
		}

		queueMessage smsg;
		smsg.mX=tempX;
		smsg.mY=tempY;
		smsg.mZ=tempZ;

		smsg.maxSpeed=100;
		smsg.timing=0;

		osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);
	}

	return 0;
}

int cmd_read(int len, char* cmd){
	return 1;
}


int cmd_handler(char* cmd)
{
	struct Command_List* pCmdList = CmdList;

	uint8_t command_found = 0;
	int read_command_found = 0;

	int len = strlen(cmd)-1;


	while (pCmdList->cmd)
	{
		if (pCmdList->cmd==cmd[0])
		{
			command_found = 1;
			read_command_found = pCmdList->func(len, ++cmd);
			break;
		}
		++pCmdList;
	}

	if (command_found == 0) printf("command not found!\n");

	return read_command_found;
}



void upEndEffector(){

//	queueMessage smsg;
//	smsg.mX=0;
//	smsg.mY=0;
//	smsg.mZ=-256.984;
//	smsg.maxSpeed=100;
//	smsg.timing=0;
//
//	osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);

	setGoalPosition(AX_BROADCAST_ID, 510);
	servoDelay(1000);
}

void downEndEffector(){
	queueMessage smsg;
	smsg.mX=0;
	smsg.mY=0;
	smsg.mZ=-407.891;

	smsg.maxSpeed=100;
	smsg.timing=0;

	osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);
}

void torqueOn(){
	syncWriteTorqueOnOff(ON);
}
void torqueOff(){
	syncWriteTorqueOnOff(OFF);
}

void pumpOn(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
}

void pumpOff(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
}

void throw(){
	queueMessage smsg;
	smsg.mX=0;
	smsg.mY=-140;
	smsg.mZ=-230;
	smsg.maxSpeed=1000;
	smsg.timing=2;

	osMessagePut(setQueueHandle, (uint32_t)&smsg, 100);
}

void cvbeltTurnRight(){
	setEndless(AX_CONVEYOR_ID, ON);
	turn(AX_CONVEYOR_ID, RIGHT, 600);
}

void cvbeltTurnLeft(){
	setEndless(AX_CONVEYOR_ID, ON);
	turn(AX_CONVEYOR_ID, LEFT, 600);
}

void cvbeltStop(){
	onOffTorque(AX_CONVEYOR_ID, OFF);
}

void deltaInit(){
	setMovingSpeed(AX_BROADCAST_ID, 100);
	upEndEffector();

//	uint8_t str[] = "******* CONTROL MENU *******\r\n 1. UP\r\n 2. DOWN\r\n 3. Read Position\r\n 4. Torque Off\r\n 5. Torque On\r\n 6 : Throw(temp)\r\n****************************\r\n";
//	HAL_UART_Transmit(&huart3, str, sizeof(str), 1000);
}
