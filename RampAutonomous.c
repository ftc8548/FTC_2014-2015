#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     				tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor, 		tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     rightWheel,    				tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     liftMotor,     				tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    dropServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    clampServoR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    clampServoL,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    irServo,              tServoStandard)

#include "includes.h"
#include "autoFunctions.h"

const int power = 100;
const int neg_power = -100;

void turnR() {
	motor[leftWheel] = power;
	motor[rightWheel] = -power;
}

void turnL() {
	motor[leftWheel] = -power;
	motor[rightWheel] = power;
}


void setPower(int power) {
	motor[leftWheel] = power;
	motor[rightWheel] = power;
}

void stopDrive() {
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
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

void slightTurnL(int turnPower) {
	motor[leftWheel] = -turnPower;
}

task main() {
	servoPrep();
	Joystick_WaitForStart();
	while(true) {
		Task_Spawn(a_raiseIR);
		slightTurnL(20);
		wait1Msec(300);
		stopDrive();
		wait1Msec(900);

		setPower(-70);
		wait1Msec(1500);
		setPower(-30);
		wait1Msec(100);
		stopDrive();
		wait1Msec(900);
		turnL();
		wait1Msec(300);
		stopDrive();
		wait1Msec(900);
		setPower(neg_power);
		wait1Msec(900);
		stopDrive();
		wait1Msec(900);
		turnR();
		wait1Msec(600);
		stopDrive();
		wait1Msec(900);
		setPower(-70);
		wait1Msec(1000);
		stopDrive();
		wait1Msec(900);
		setPower(-30);
		wait1Msec(1000);
		stopDrive();
		wait1Msec(900);
		Task_Spawn(a_dropClamp);
		wait1Msec(900);
		setPower(30);
		wait1Msec(500);
		stopDrive();
		wait1Msec(900);
		raiseLift();
		wait1Msec(2800);
		stopLift();
		wait1Msec(900);
		Task_Spawn(a_dropBall);
		wait1Msec(3000);
		Task_Spawn(a_resetDrop);
		lowerLift();
		wait1Msec(4000);
		stopLift();
		setPower(100);
		wait1Msec(600);
		stopDrive();
		wait1Msec(900);
		turnL();
		wait1Msec(300);
		stopDrive();
		wait1Msec(900);
		setPower(100);
		wait1Msec(2100);
		stopDrive();
		wait1Msec(900);
		turnR();
		wait1Msec(600);
		stopDrive();
		wait1Msec(900);
		setPower(30);
		wait1Msec(1000);
		stopDrive();
		wait1Msec(900);

		break;
	}
}
