#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     pickupMotor,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     Blah,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_1,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    irServo,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"
#include "functions.h"

task main() {
	Joystick_WaitForStart();
	Task_Spawn(Gyro);
	Task_Spawn(raiseIR);
	Task_Spawn(readIR);

	motor[rightWheel] = 100;
	motor[leftWheel] = 100;
	wait1Msec(1250);
	motor[rightWheel] = 0;
	motor[leftWheel] = 0;
	wait1Msec(4000);
	driveForward(40);
	turnLeft(90.0);
	turnRight(180.0);
	driveForward(20);
}
