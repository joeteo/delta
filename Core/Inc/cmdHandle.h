/*
 * cli.h
 *
 *  Created on: Sep 4, 2022
 *      Author: J
 */


#ifndef INC_CLI_H_
#define INC_CLI_H_

#define MAX_CMD_NUM 10


typedef struct
{
	float mX;
	float mY;
	float mZ;
	int maxSpeed;

}queueMessage;


typedef int cmd_func(int len, char* cmd, queueMessage* smsg);

struct Command_List
{
	char cmd;
	cmd_func* func;
};

int cmd_torque(int len, char* cmd, queueMessage* smsg);
int cmd_pump(int len, char* cmd, queueMessage* smsg);
int cmd_conveyorBelt(int len, char* cmd, queueMessage* smsg);
int cmd_pick(int len, char* cmd, queueMessage* smsg);
int cmd_throw(int len, char* cmd, queueMessage* smsg);
int cmd_defaultPos(int len, char* cmd, queueMessage* smsg);
int cmd_read(int len, char* cmd, queueMessage* smsg);
int cmd_moveTo(int len, char* cmd, queueMessage* smsg);

int cmd_handler(char* cmd, queueMessage* smsg);

void deltaInit();
void upEndEffector(queueMessage* smsg);
void downEndEffector(queueMessage* smsg);
void torqueOn();
void torqueOff();
void pumpOn();
void pumpOff();
void cvbeltTurnRight();
void cvbeltTurnLeft();
void cvbeltStop();


#endif /* INC_CLI_H_ */
