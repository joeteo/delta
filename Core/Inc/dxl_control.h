/*
 * dxl_control.h
 *
 *  Created on: Aug 11, 2022
 *      Author: Soohyun Jo
 */

#include "main.h"



#ifndef INC_DXL_CONTROL_H_
#define INC_DXL_CONTROL_H_

#define AX_HEADER					0xFF
#define AX_BROADCAST_ID				0xFE
#define AX_CONVEYOR_ID				3

#define	AX_WRITE					0x03
#define	AX_READ						0x02
#define	AX_SYNC_WRITE				0x83
#define AX_READ_ONE_BYTE            1
#define AX_READ_TWO_BYTE            2

#define	OFF							0
#define	ON							1
#define	RX_MODE						0
#define	TX_MODE						1
#define	LEFT						0
#define	RIGHT						1

	// EEPROM AREA
#define ADDR_MODEL_NUMBER_L           0
#define ADDR_MODEL_NUMBER_H           1
#define ADDR_VERSION                  2
#define ADDR_ID                       3
#define ADDR_BAUD_RATE                4
#define ADDR_RETURN_DELAY_TIME        5
#define ADDR_CW_ANGLE_LIMIT_L         6
#define ADDR_CW_ANGLE_LIMIT_H         7
#define ADDR_CCW_ANGLE_LIMIT_L        8
#define ADDR_CCW_ANGLE_LIMIT_H        9
#define ADDR_SYSTEM_DATA2             10
#define ADDR_LIMIT_TEMPERATURE        11
#define ADDR_DOWN_LIMIT_VOLTAGE       12
#define ADDR_UP_LIMIT_VOLTAGE         13
#define ADDR_MAX_TORQUE_L             14
#define ADDR_MAX_TORQUE_H             15
#define ADDR_RETURN_LEVEL             16
#define ADDR_ALARM_LED                17
#define ADDR_ALARM_SHUTDOWN           18
#define ADDR_OPERATING_MODE           19
#define ADDR_DOWN_CALIBRATION_L       20
#define ADDR_DOWN_CALIBRATION_H       21
#define ADDR_UP_CALIBRATION_L         22
#define ADDR_UP_CALIBRATION_H         23

	// RAM AREA
#define ADDR_TORQUE_ENABLE            24
#define ADDR_LED                      25
#define ADDR_CW_COMPLIANCE_MARGIN     26
#define ADDR_CCW_COMPLIANCE_MARGIN    27
#define ADDR_CW_COMPLIANCE_SLOPE      28
#define ADDR_CCW_COMPLIANCE_SLOPE     29
#define ADDR_GOAL_POSITION_L	      30
#define ADDR_GOAL_POSITION_H          31
#define ADDR_GOAL_SPEED_L             32
#define ADDR_GOAL_SPEED_H             33
#define ADDR_TORQUE_LIMIT_L           34
#define ADDR_TORQUE_LIMIT_H           35
#define ADDR_PRESENT_POSITION_L       36
#define ADDR_PRESENT_POSITION_H       37
#define ADDR_PRESENT_SPEED_L          38
#define ADDR_PRESENT_SPEED_H          39
#define ADDR_PRESENT_LOAD_L           40
#define ADDR_PRESENT_LOAD_H           41
#define ADDR_PRESENT_VOLTAGE          42
#define ADDR_PRESENT_TEMPERATURE      43
#define ADDR_REGISTERED_INSTRUCTION   44
#define ADDR_PAUSE_TIME               45
#define ADDR_MOVING                   46
#define ADDR_LOCK                     47
#define ADDR_PUNCH_L                  48
#define ADDR_PUNCH_H                  49


typedef enum{
	FALSE	= 0,
	TRUE	= 1
}bool;

void deltaInit();
void sendInstPacket(uint8_t* packet, uint8_t length);
void servoDelay(uint32_t millisec);
void setMovingSpeed(uint8_t ID, uint16_t Speed);
void setGoalPosition(uint8_t ID, uint16_t Position);
void onOffTorque(uint8_t ID, uint8_t State);
uint16_t getPresentPosition(uint8_t ID);
void upEndEffector();
void downEndEffector();
void syncWriteGoalPosition(uint16_t P0, uint16_t S0, uint16_t P1, uint16_t S1, uint16_t P2, uint16_t S2);
void setEndless(uint8_t ID, uint8_t State);
void turn(uint8_t ID, uint8_t SIDE, uint16_t Speed);
void syncWriteTorqueOnOff(uint8_t State);


#endif /* INC_DXL_CONTROL_H_ */
