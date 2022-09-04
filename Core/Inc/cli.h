/*
 * cli.h
 *
 *  Created on: Sep 4, 2022
 *      Author: J
 */


#ifndef INC_CLI_H_
#define INC_CLI_H_

#define MAX_CMD_NUM 10


typedef int cmd_func(int argc, char* argv[]);

struct Command_List
{
	char* cmd;
	cmd_func* func;
	char* help_str;
};

int cmd_torque(int argc, char* argv[]);
int cdm_pump(int argc, char* argv[]);
int cmd_conveyorBelt(int argc, char* argv[]);
int cmd_pick(int argc, char* argv[]);
int cmd_throw(int argc, char* argv[]);
void parse_input_string(char* input_string, int* argc, char* argv[]);
void cmd_handler(char* cmd);




#endif /* INC_CLI_H_ */
