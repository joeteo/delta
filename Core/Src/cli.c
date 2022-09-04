/*
 * cli.c
 *
 *  Created on: Sep 4, 2022
 *      Author: Jo soo hyun
 *
 *
 */

#include "cli.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart3;

#include "cmsis_os.h"

extern osSemaphoreId UPSemHandle;
extern osSemaphoreId DOWMSemHandle;
extern osSemaphoreId ReadPosSemHandle;
extern osSemaphoreId TorqueOnSemHandle;
extern osSemaphoreId TorqueOffSemHandle;
extern osSemaphoreId ThrowSemHandle;
extern osSemaphoreId PumpOnSemHandle;
extern osSemaphoreId PumpOffSemHandle;
extern osSemaphoreId ConveyorRightOnSemHandle;
extern osSemaphoreId ConveyorLeftOnSemHandle;
extern osSemaphoreId ConveyorOffSemHandle;


int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart3, (uint8_t*)p, len, 10);
	return len;
}

struct Command_List  CmdList[] =
{
	{"torque",	cmd_torque,	"on/off Torque"},
	{"pump",	cdm_pump,	"on/off Pump"},
	{"cvbelt",	cmd_conveyorBelt,	"on/off Conveyor Belt. right/left"},
	{"pick",	cmd_pick,	"no option. End effector Down + Pump On + default position"},
	{"throw",	cmd_throw,	"no option. Throw + default position"},
	{0,0,0}
};



int cmd_torque(int argc, char* argv[])
{
	if (argv[1] == NULL) {
		printf("wrong command pattern. \n");
	}
	else {
		if (!strcmp(argv[1], "on")) {
			osSemaphoreRelease(TorqueOnSemHandle);
		}
		else if (!strcmp(argv[1], "off")) {
			osSemaphoreRelease(TorqueOffSemHandle);
		}
	}

	return 0;
}

int cdm_pump(int argc, char* argv[])
{
	if (argv[1] == NULL) {
		printf("wrong command pattern. \n");
	}
	else {
		if (!strcmp(argv[1], "on")) {
			osSemaphoreRelease(PumpOnSemHandle);
		}
		else if (!strcmp(argv[1], "off")) {
			osSemaphoreRelease(PumpOffSemHandle);
		}
	}

	return 0;
}

int cmd_conveyorBelt(int argc, char* argv[])
{
	if (argv[1] == NULL) {
		printf("wrong command pattern. \n");
	}
	else {
		if (!strcmp(argv[1], "on")) {
			if (argv[2] == NULL) {
				printf("wrong command pattern. \n");
			}
			else {
				if (!strcmp(argv[2], "right")) {
					osSemaphoreRelease(ConveyorRightOnSemHandle);
				}
				else if (!strcmp(argv[2], "left")) {
					osSemaphoreRelease(ConveyorLeftOnSemHandle);
				}else {
					printf("wrong command pattern. \n");
				}
			}
		}
		else if (!strcmp(argv[1], "off")) {
			osSemaphoreRelease(ConveyorOffSemHandle);
		}
	}

	return 0;
}

int cmd_pick(int argc, char* argv[])
{
	printf("pick up item on the conveyor belt. \n");
	return 0;
}

int cmd_throw(int argc, char* argv[])
{
	printf("throw item away \n");
	return 0;
}


void parse_input_string(char* input_string, int* argc, char* argv[])
{
	int found_arg = 1;
	int argn = 0;

	while (*input_string)
	{
		if (*input_string == '\n') {
			*input_string = '\0';
			break;
		}

		if (*input_string == ' ') {
			found_arg = 1;
			*input_string = '\0';
		}
		else if (found_arg) {
			argv[argn++] = input_string;
			found_arg = 0;
		}
		input_string++;
	}

	*argc = argn;
}


void cmd_handler(char* cmd)
{
	struct Command_List* pCmdList = CmdList;

	uint32_t command_found = 0;

	int	  argc;
	char* argv[MAX_CMD_NUM] = { 0, };

	parse_input_string(cmd, &argc, argv);

	if (argc)
	{
		while (pCmdList->cmd)
		{
			if (!strcmp((const char*)pCmdList->cmd, (const char*)cmd))
			{
				command_found = 1;
				pCmdList->func(argc, argv);
				break;
			}
			++pCmdList;
		}
	}
	if (command_found == 0) printf("command not found!\n");
}


