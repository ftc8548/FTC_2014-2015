#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     gyroSensor,     sensorAnalogInactive)
#pragma config(Sensor, S3,     	 irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     				tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor, 		tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     rightWheel,    				tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     liftMotor,     				tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    dropServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    clampServoR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    clampServoL,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    irServo,              tServoStandard)

#include "includes.h"
#include "autoFunctions.h"

void TR(int waitTime) {
	motor[rightWheel] = 40;
	motor[leftWheel] = -40;
	wait1Msec(waitTime);
	motor[rightWheel] = 0;
	motor[leftWheel] = 0;
}

void TL(int waitTime) {
	motor[rightWheel] = -40;
	motor[leftWheel] = 40;
	wait1Msec(waitTime);
	motor[rightWheel] = 0;
	motor[leftWheel] = 0;
}

void DB(int waitTime) {
	motor[rightWheel] = -20;
	motor[leftWheel] = -20;
	wait1Msec(waitTime);
	motor[rightWheel] = 0;
	motor[leftWheel] = 0;
}

void raiseLift() {
	motor[liftMotor] = 100;
}

void lowerLift() {
	motor[liftMotor] = -30;
}

void stopLift() {
	motor[liftMotor] = 0;
}

task main() {
	servoPrep();
	//Joystick_WaitForStart();
	disableDiagnosticsDisplay();
	while(true) {
		encoderPrep();
		startTrackers();
		turnRight(90.0);
		nxtDisplayTextLine(4, "DONE");
		////ramp(600.0);
		////turnRight(130.0); gives 90
		//driveBackward(100.0);
		//wait1Msec(550);
		//turnRight(160.0);
		//wait1Msec(550);
		//driveBackward(150.0);
		//wait1Msec(550);
		//TR(675);
		//wait1Msec(800);
		//driveBackward(775.0);
		//wait1Msec(550);
		//TL(700);
		//wait1Msec(550);
		//ramp(125);
		//wait1Msec(550);
		//DB(150);
		//wait1Msec(550);
		//Task_Spawn(a_dropClamp);
		//wait1Msec(550);
		//raiseLift();
		//wait1Msec(1800);
		//stopLift();
		//wait1Msec(900);
		//Task_Spawn(a_dropBall);
		//wait1Msec(3000);
		//Task_Spawn(a_resetDrop);
		//lowerLift();
		while (true) {
			wait1Msec(3000);
		}
		//stopLift();
		//break;
	}
}
