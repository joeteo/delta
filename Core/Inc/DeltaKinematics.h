/*
 * DeltaKinematics.h
 *
 *  Created on: 2022. 8. 15.
 *      Author: J
 */

#include "main.h"

#ifndef INC_DELTAKINEMATICS_H_
#define INC_DELTAKINEMATICS_H_


typedef struct {
    double x;
    double y;
    double z;
}_Coordinates;

#define non_existing_povar_error -2
#define no_error 0

void ServoConversion();
double* ConversionFromServo(uint16_t PP0, uint16_t PP1, uint16_t PP2);

void setCoordinates(double x, double y, double z);
void resetCoordinates(double x, double y, double z);


int inverse();
int delta_calcAngleYZ(double* Angle, double x0, double y0, double z0);

int forward(double thetaA, double thetaB, double thetaC);




#endif /* INC_DELTAKINEMATICS_H_ */
