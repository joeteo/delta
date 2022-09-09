/*
 * cli.h
 *
 *  Created on: Sep 4, 2022
 *      Author: J
 */


#ifndef INC_CLI_H_
#define INC_CLI_H_

#define MAX_CMD_NUM 10


typedef int cmd_func(int len, char* cmd);

struct Command_List
{
	char cmd;
	cmd_func* func;
};

int cmd_torque(int len, char* cmd);
int cdm_pump(int len, char* cmd);
int cmd_conveyorBelt(int len, char* cmd);
int cmd_pick(int len, char* cmd);
int cmd_throw(int len, char* cmd);
int cmd_defaultPos(int len, char* cmd);
int cmd_read(int len, char* cmd);
int cmd_moveTo(int len, char* cmd);

int cmd_handler(char* cmd);

typedef struct
{
	float mX;
	float mY;
	float mZ;
	int maxSpeed;
	unsigned char timing;

}queueMessage;

void deltaInit();
void upEndEffector();
void downEndEffector();
void torqueOn();
void torqueOff();
void pumpOn();
void pumpOff();
void throw();
void cvbeltTurnRight();
void cvbeltTurnLeft();
void cvbeltStop();


#endif /* INC_CLI_H_ */
