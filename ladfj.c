#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     L,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     R,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     R,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    irServo,              tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"

task main()
{
	Joystick_WaitForStart();
motor[R] = 100;
motor[L] = -100;
wait1Msec(1000);
motor[R] = 0;
motor[L] = 0;
servo[irServo] = 100;
wait1Msec(10000);
}