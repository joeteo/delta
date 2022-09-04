/*
 * DeltaKinematics.c
 *
 *  Created on: 2022. 8. 15.
 *      Author: J
 */

#include "DeltaKinematics.h"

 /**********************************************************************************************
  * Thanks to Arduino Library : https://github.com/tinkersprojects/Delta-Kinematics-Library
  **********************************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>


const double BicepLength = 180;
const double ForearmLength = 320;
const double BaseRadius = 67;
const double EndEffectorRadius = 30;

#define sqrt3 1.7320508075688773
#define pi M_PI
#define sin120 sqrt3 / 2.0
#define cos120 -0.5
#define tan60 sqrt3
#define sin30 0.5
#define tan30 1.0 / sqrt3

_Coordinates C;

double ThetaA;
double ThetaB;
double ThetaC;

uint16_t GP[3];

double coord[3];

void ServoConversion() {
    GP[0] = (uint16_t)(((ThetaA + 147.9) / 0.29)+0.5);
    GP[1] = (uint16_t)(((ThetaB + 147.9) / 0.29)+0.5);
    GP[2] = (uint16_t)(((ThetaC + 147.9) / 0.29)+0.5);
}

double* ConversionFromServo(uint16_t PP0, uint16_t PP1, uint16_t PP2) {
	static double theta[3];

	theta[0]=0.0;
	theta[1]=0.0;
	theta[2]=0.0;

	theta[0]=(PP0*0.29)-147.9;
	theta[1]=(PP1*0.29)-147.9;
	theta[2]=(PP2*0.29)-147.9;

	return theta;

}

void setCoordinates(double x, double y, double z) {
    C.x = x;
    C.y = y;
    C.z = z;
}

void resetCoordinates(double x, double y, double z) {
    C.x += x;
    C.y += y;
    C.z += z;
}

/******************* SETUP *******************/


// inverse kinematics
// helper functions, calculates angle thetaA (for YZ-pane)
int delta_calcAngleYZ(double* Angle, double x0, double y0, double z0)
{
    double y1 = -0.5 * 0.57735 * BaseRadius;  // f/2 * tan(30 deg)
    y0 -= 0.5 * 0.57735 * EndEffectorRadius;  // shift center to edge

// z = a + b*y
    double aV = (x0 * x0 + y0 * y0 + z0 * z0 + BicepLength * BicepLength - ForearmLength * ForearmLength - y1 * y1) / (2.0 * z0);
    double bV = (y1 - y0) / z0;

    // discriminant
    double dV = -(aV + bV * y1) * (aV + bV * y1) + BicepLength * (bV * bV * BicepLength + BicepLength);
    if (dV < 0)
    {
        return non_existing_povar_error; // non-existing povar.  return error, theta
    }

    double yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1); // choosing outer povar
    double zj = aV + bV * yj;
    *Angle = atan2(-zj, (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);

    return no_error;  // return error, theta
}


// inverse kinematics: (x0, y0, z0) -> (thetaA, thetaB, thetaC)

int inverse()
{
    ThetaA = 0;
    ThetaB = 0;
    ThetaC = 0;

    int error = delta_calcAngleYZ(&ThetaA, C.x, C.y, C.z);
    if (error != no_error)
        return no_error;
    error = delta_calcAngleYZ(&ThetaB, C.x * cos120 + C.y * sin120, C.y * cos120 - C.x * sin120, C.z);
    if (error != no_error)
        return no_error;
    error = delta_calcAngleYZ(&ThetaC, C.x * cos120 - C.y * sin120, C.y * cos120 + C.x * sin120, C.z);

    return no_error;


}

//forward kinematics: (thetaA, thetaB, thetaC) -> (x0, y0, z0)
int forward(double theta1, double theta2, double theta3)
{

	  coord[0]=0.0;
	  coord[1]=0.0;
	  coord[2]=0.0;

	  double t = (BaseRadius-EndEffectorRadius)*tan30/2.0;
	  double dtr = pi/180.0;

	  theta1 *= dtr;
	  theta2 *= dtr;
	  theta3 *= dtr;

	  double y1 = -(t + BicepLength*cos(theta1));
	  double z1 = -BicepLength*sin(theta1);

	  double y2 = (t + BicepLength*cos(theta2))*sin30;
	  double x2 = y2*tan60;
	  double z2 = -BicepLength*sin(theta2);

	  double y3 = (t + BicepLength*cos(theta3))*sin30;
	  double x3 = -y3*tan60;
	  double z3 = -BicepLength*sin(theta3);

	  double dnm = (y2-y1)*x3-(y3-y1)*x2;

	  double w1 = y1*y1 + z1*z1;
	  double w2 = x2*x2 + y2*y2 + z2*z2;
	  double w3 = x3*x3 + y3*y3 + z3*z3;

	  // x = (a1*z + b1)/dnm
	  double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	  double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

	  // y = (a2*z + b2)/dnm;
	  double a2 = -(z2-z1)*x3+(z3-z1)*x2;
	  double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

	  // a*z^2 + b*z + c = 0
	  double a = a1*a1 + a2*a2 + dnm*dnm;
	  double b = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	  double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - ForearmLength*ForearmLength);

	  // discriminant
	  double d = b*b - 4.0*a*c;
	  if (d < 0.0) return non_existing_povar_error; // non-existing povar. return error,x,y,z



	  coord[2] = -0.5*(b+sqrt(d))/a;
	  coord[0] = (a1*coord[2] + b1)/dnm;
	  coord[1] = (a2*coord[2] + b2)/dnm;

	  return no_error;

//    double t = (BaseRadius - EndEffectorRadius) * tan30 / 2.0;
//    double dtr = pi / 180.0;
//
//    tA *= dtr;
//    tB *= dtr;
//    tC *= dtr;
//
//    double y1 = -(t + BicepLength * cos(tA));
//    double z1 = -BicepLength * sin(tA);
//
//    double y2 = (t + BicepLength * cos(tB)) * sin30;
//    double x2 = y2 * tan60;
//    double z2 = -BicepLength * sin(tB);
//
//    double y3 = (t + BicepLength * cos(tC)) * sin30;
//    double x3 = -y3 * tan60;
//    double z3 = -BicepLength * sin(tC);
//
//    double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
//
//    double w1 = y1 * y1 + z1 * z1;
//    double w2 = x2 * x2 + y2 * y2 + z2 * z2;
//    double w3 = x3 * x3 + y3 * y3 + z3 * z3;
//
//    // x = (a1*z + b1)/dnm
//    double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
//    double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;
//
//    // y = (a2*z + b2)/dnm;
//    double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
//    double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;
//
//    // a*z^2 + b*z + c = 0
//    double aV = a1 * a1 + a2 * a2 + dnm * dnm;
//    double bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
//    double cV = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - ForearmLength * ForearmLength);
//
//    // discriminant
//    double dV = bV * bV - 4.0 * aV * cV;
//    if (dV < 0.0)
//    {
//        return non_existing_povar_error; // non-existing povar. return error,x,y,z
//    }
//
//    C.z = (a2 * C.z + b2) / dnm;
//    C.x = -0.5 * (bV + sqrt(dV)) / aV;
//    C.y = (a1 * C.z + b1) / dnm;
//
//
//    return no_error;
}


