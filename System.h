#ifndef SYSTEM_H
#define SYSTEM_H

#include "IMCDriver.h"
#include "IMCDefine.h"
#include <stdio.h>

const int SERVOLOOP_TIME = 1;	// unit: ms   (Set Servo Loop Interrupt time)
const float REAL_TIMER = SERVOLOOP_TIME / 1000.0;	// unit: s

const float PI = 3.1415926;

const int ARM_L = 0;
const int ARM_R = 1;

const float Mg1 = 19.6;	//2*9.8
const float Mg2 = 19.6;	//2*9.8
const float L1 = 0.3; // 臂長
const float L2 = 0.3; // 臂長

const int REDUCER50 = 50; // 減速比50
const int REDUCER100 = 100; //減速比100

const int MOTOR_ENCODER_4X[6] = {2000, 2000, 2048, 2048, 2048, 2048}; // 編碼器之值

const int MAX_OUTPUT_VOLTAGE = 10; //最大輸出電壓
const int MIN_OUTPUT_VOLTAGE = -10; //最小輸出電壓

struct MOTOR_DATA{       // 馬達之基本參數
	float nominal_torque;// nominal torque (max. continuous torque)
	float R;			// terminal resistor [Ohm]
	float Km;			// torque constant [Nm/A]
	float Kb;			// back EMF coefficient [V/(rad/s)]
	float Im;			// rotor inertia [kg*cm^2]
	float Cm;			// viscosity coefficient of the system load
};

const MOTOR_DATA M_DATA[6]=   //六顆馬達之基本參數(nominal torque,R,Km,Kb,Im,Cm)
{
	{ 0.354,  0.113, 0.0396, 60/(241*PI), 0.584,  0.1251700969},
	{ 0.354,  0.113, 0.0396, 60/(241*PI), 0.584,  0.1251700969},
	{ 0.17,   0.317, 0.0302, 60/(317*PI), 0.138,  0.0258131 },
	{ 0.17,   0.317, 0.0302, 60/(317*PI), 0.138,  0.0258131 },
	{ 0.0288, 1.64,  0.0208, 60/(460*PI), 0.0138, 0.0258131 },
	{ 0.0288, 1.64,  0.0208, 60/(460*PI), 0.0138, 0.0258131 }
};
const float OUTPUT_TORQUE_FACTOR[2][6]=
{
	{	MAX_OUTPUT_VOLTAGE / M_DATA[0].nominal_torque / REDUCER50,
		MAX_OUTPUT_VOLTAGE / M_DATA[1].nominal_torque / REDUCER50 * (-1),
		MAX_OUTPUT_VOLTAGE / M_DATA[2].nominal_torque / REDUCER50,
		MAX_OUTPUT_VOLTAGE / M_DATA[3].nominal_torque / REDUCER50,
		MAX_OUTPUT_VOLTAGE / M_DATA[4].nominal_torque / REDUCER100,
		MAX_OUTPUT_VOLTAGE / M_DATA[5].nominal_torque / REDUCER100	      },

	{	MAX_OUTPUT_VOLTAGE / M_DATA[0].nominal_torque / REDUCER100,
		MAX_OUTPUT_VOLTAGE / M_DATA[1].nominal_torque / REDUCER100 * (-1),
		MAX_OUTPUT_VOLTAGE / M_DATA[2].nominal_torque / REDUCER100,
		MAX_OUTPUT_VOLTAGE / M_DATA[3].nominal_torque / REDUCER100,
		MAX_OUTPUT_VOLTAGE / M_DATA[4].nominal_torque / REDUCER100,
		MAX_OUTPUT_VOLTAGE / M_DATA[5].nominal_torque / REDUCER100	      }
};

const float ENC2RAD_FACTOR[2][6]=
{
	{	2*PI / MOTOR_ENCODER_4X[0] / REDUCER50 * (-1),
		2*PI / MOTOR_ENCODER_4X[1] / REDUCER50,
		2*PI / MOTOR_ENCODER_4X[2] / REDUCER50 * (-1),
		2*PI / MOTOR_ENCODER_4X[3] / REDUCER50 * (-1),
		2*PI / MOTOR_ENCODER_4X[4] / REDUCER100,
		2*PI / MOTOR_ENCODER_4X[5] / REDUCER100,			},
	
	{	2*PI / MOTOR_ENCODER_4X[0] / REDUCER100 * (-1),
		2*PI / MOTOR_ENCODER_4X[1] / REDUCER100,
		2*PI / MOTOR_ENCODER_4X[2] / REDUCER100 * (-1),
		2*PI / MOTOR_ENCODER_4X[3] / REDUCER100 * (-1),
		2*PI / MOTOR_ENCODER_4X[4] / REDUCER100,
		2*PI / MOTOR_ENCODER_4X[5] / REDUCER100,			}
};

//系統之函式原型
void Initial_IMPCard( int Card_Index );
void Initial_Encoder( int Card_Index );
void Initial_DAC( int Card_Index );
void Read_Encoder( float joint_pos[2][6], long lEncoder[2][6], int Card_Index );
void Output_Voltage( float updateTorque[2][6], int Card_Index );
void Close_IMPCard( int Card_Index );

#endif