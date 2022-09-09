/*
 * dxl_control.c
 *
 *  Created on: Aug 11, 2022
 *      Author: Soohyun Jo
 *      thanks to Arduino Library : https://github.com/jumejume1/AX-12A-servo-library
 */

#include "dxl_control.h"
#include "cmsis_os.h"
#include "DeltaKinematics.h"
#include <stdio.h>
#include <string.h>


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;



extern _Coordinates C;

extern double ThetaA;
extern double ThetaB;
extern double ThetaC;

extern uint16_t GP[3];


#define RxBuf_SIZE 20
extern uint8_t rx2_Buf[RxBuf_SIZE];

char buffer[10]={0,};


void servoDelay(uint32_t millisec){
	osDelay(millisec);
}

void sendInstPacket(uint8_t* packet, uint8_t length)
{
	HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, TX_MODE); // Switch to Transmission  Mode

	HAL_UART_Transmit(&huart2, packet, length, 1000);
	//servoDelay(25);

	HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, RX_MODE); 	// Switch back to Reception Mode

}

void setMovingSpeed(uint8_t ID, uint16_t Speed)
{
    uint8_t Speed_L = Speed;
    uint8_t Speed_H = Speed >> 8;
    // 16 bits -> 2 x 8 bits

    uint8_t length = 9;
    uint8_t packet[length];

    packet[0] = AX_HEADER;
    packet[1] = AX_HEADER;
    packet[2] = ID;
    packet[3] = length-4;
    packet[4] = AX_WRITE;
    packet[5] = ADDR_GOAL_SPEED_L;
    packet[6] = Speed_L;
    packet[7] = Speed_H;
    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])) & 0xFF;
    packet[8] = Checksum;

    sendInstPacket(packet, length);

}

void setGoalPosition(uint8_t ID, uint16_t Position)
{
    uint8_t Position_L = Position;
    uint8_t Position_H = Position >> 8;
    // 16 bits -> 2 x 8 bits

    uint8_t length = 9;
    uint8_t packet[length];

    packet[0] = AX_HEADER;
    packet[1] = AX_HEADER;
    packet[2] = ID;
    packet[3] = length-4;
    packet[4] = AX_WRITE;
    packet[5] = ADDR_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])) & 0xFF;
    packet[8] = Checksum;

    sendInstPacket(packet, length);

}

void syncWriteGoalPosition(uint16_t P0, uint16_t S0, uint16_t P1, uint16_t S1, uint16_t P2, uint16_t S2){

    uint8_t P0_L = P0;
    uint8_t P0_H = P0 >> 8;

    uint8_t P1_L = P1;
    uint8_t P1_H = P1 >> 8;

    uint8_t P2_L = P2;
    uint8_t P2_H = P2 >> 8;

    uint8_t S0_L = S0;
    uint8_t S0_H = S0 >> 8;

    uint8_t S1_L = S1;
    uint8_t S1_H = S1 >> 8;

    uint8_t S2_L = S2;
    uint8_t S2_H = S2 >> 8;

    uint8_t length = 23;
    uint8_t packet[length];

    packet[0] = AX_HEADER;
    packet[1] = AX_HEADER;
    packet[2] = AX_BROADCAST_ID;
    packet[3] = length-4;
    packet[4] = AX_SYNC_WRITE;
    packet[5] = ADDR_GOAL_POSITION_L;
    packet[6] = 0x04;		// length of data to access

    packet[7] = 0;			// ID 0
    packet[8] = P0_L;
    packet[9] = P0_H;
    packet[10] = S0_L;
    packet[11] = S0_H;

    packet[12] = 1;			// ID 1
    packet[13] = P1_L;
    packet[14] = P1_H;
    packet[15] = S1_L;
    packet[16] = S1_H;

    packet[17] = 2;			// ID 2
    packet[18] = P2_L;
    packet[19] = P2_H;
    packet[20] = S2_L;
    packet[21] = S2_H;
    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]
						+ packet[8] + packet[9] + packet[10] + packet[11] + packet[12] + packet[13]
						+ packet[14] + packet[15] + packet[16] + packet[17] + packet[18] + packet[19]
						+ packet[20] + packet[21])) & 0xFF;
    packet[22] = Checksum;

    sendInstPacket(packet, length);
}

uint16_t getPresentPosition(uint8_t ID)
{

    uint8_t length = 8;
    uint8_t packet[length];

    packet[0] = AX_HEADER;
    packet[1] = AX_HEADER;
    packet[2] = ID;
    packet[3] = length-4;
    packet[4] = AX_READ;
    packet[5] = ADDR_PRESENT_POSITION_L;
    packet[6] = AX_READ_TWO_BYTE;
    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6])) & 0xFF;
    packet[7] = Checksum;

    sendInstPacket(packet, length);

    servoDelay(10);
    Checksum = (~(rx2_Buf[2] + rx2_Buf[3] + rx2_Buf[4] + rx2_Buf[5] + rx2_Buf[6])) & 0xFF;
    uint16_t presentPosition = rx2_Buf[5] + (rx2_Buf[6]<<8);

    if(Checksum==rx2_Buf[7]){
    	return presentPosition;
    }else {
    	return 0;
    }

}


void onOffTorque(uint8_t ID, uint8_t State){

    uint8_t length = 8;
    uint8_t packet[length];

    packet[0] = AX_HEADER;
    packet[1] = AX_HEADER;
    packet[2] = ID;
    packet[3] = length-4;
    packet[4] = AX_WRITE;
    packet[5] = ADDR_TORQUE_ENABLE;
    packet[6] = State;
    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6])) & 0xFF;
    packet[7] = Checksum;

    sendInstPacket(packet, length);

}

void syncWriteTorqueOnOff(uint8_t State){

    uint8_t length = 14;
    uint8_t packet[length];

    packet[0] = AX_HEADER;
    packet[1] = AX_HEADER;
    packet[2] = AX_BROADCAST_ID;
    packet[3] = length-4;
    packet[4] = AX_SYNC_WRITE;
    packet[5] = ADDR_TORQUE_ENABLE;
    packet[6] = 0x01;		// length of data to access

    packet[7] = 0;			// ID 0
    packet[8] = State;

    packet[9] = 1;			// ID 1
    packet[10] = State;

    packet[11] = 2;			// ID 2
    packet[12] = State;
    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]
						+ packet[8] + packet[9] + packet[10] + packet[11] + packet[12])) & 0xFF;
    packet[13] = Checksum;

    sendInstPacket(packet, length);
}


void setEndless(uint8_t ID, uint8_t State)
{
	if ( State )
	{
		uint8_t length = 9;
		uint8_t packet[length];

	    packet[0] = AX_HEADER;
	    packet[1] = AX_HEADER;
	    packet[2] = ID;
	    packet[3] = length-4;
	    packet[4] = AX_WRITE;
	    packet[5] = ADDR_CCW_ANGLE_LIMIT_L;
	    packet[6] = 0; 						// full rotation
	    packet[7] = 0;						// full rotation
	    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])) & 0xFF;
	    packet[8] = Checksum;

	    sendInstPacket(packet, length);
	}
	else	// use torque off instead
	{
		turn(ID,0,0);

		uint8_t length = 9;
		uint8_t packet[length];

	    packet[0] = AX_HEADER;
	    packet[1] = AX_HEADER;
	    packet[2] = ID;
	    packet[3] = length-4;
	    packet[4] = AX_WRITE;
	    packet[5] = ADDR_CCW_ANGLE_LIMIT_L;
	    packet[6] = 255;					// 1023 low
	    packet[7] = 3;						// 1023 high
	    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])) & 0xFF;
	    packet[8] = Checksum;

	    sendInstPacket(packet, length);
	}
}

void turn(uint8_t ID, uint8_t SIDE, uint16_t Speed)
{
		if (SIDE == LEFT)
		{

		    uint8_t Speed_L = Speed;
		    uint8_t Speed_H = Speed >> 8;		// 16 bits - 2 x 8 bits variables

			uint8_t length = 9;
			uint8_t packet[length];

		    packet[0] = AX_HEADER;
		    packet[1] = AX_HEADER;
		    packet[2] = ID;
		    packet[3] = length-4;
		    packet[4] = AX_WRITE;
		    packet[5] = ADDR_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])) & 0xFF;
		    packet[8] = Checksum;

		    sendInstPacket(packet, length);
		}

		else
		{

		    uint8_t Speed_L = Speed;
		    uint8_t Speed_H = (Speed >> 8) + 4;		// 16 bits - 2 x 8 bits variables

			uint8_t length = 9;
			uint8_t packet[length];

		    packet[0] = AX_HEADER;
		    packet[1] = AX_HEADER;
		    packet[2] = ID;
		    packet[3] = length-4;
		    packet[4] = AX_WRITE;
		    packet[5] = ADDR_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    uint8_t Checksum = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])) & 0xFF;
		    packet[8] = Checksum;

		    sendInstPacket(packet, length);
		}
}









