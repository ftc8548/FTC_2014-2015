#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     liftMotor,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_1, 	secondPickupMotor,	tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    irServo,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    clampServo,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    dropServo,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//#pragma config(Motor,  mtr_S1_C4_1,     liftMotor,     tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"

/////////////////////////////// Global Variables ////////////////////

const int timeRaiseGoal1 = 1 *1000;
const int timeRaiseGoal2 = 2 *1000;
const int timeRaiseGoal3 = 3 *1000;
const int timeRaiseGoal4 = 4 *1000;
const int timeDropGoal1 = 0.5 * 1000;
const int timeDropGoal2 = 1.5 * 1000;
const int timeDropGoal3 = 2.5 * 1000;
const int timeDropGoal4 = 3.5 * 1000;
const int dropPower = -50;
bool isLift = false;
bool isPickup = false;
bool isTurning = false;
typedef enum Position {
	upLow,
	upMiddle,
	upHigh,
	upCenter,
	down
};
Position position = down;

///////////////////////////// Function Declarations ///////////////////

// raises the lift
void raiseLift(int seconds);
void raiseLift(bool isLift);
// lowers the lift
void lowerLift(int r_position);
void lowerLift(bool isLift);
// starts the pickup
void startPickup();
// reverses the pickup
void reversePickup();
// stops the pickup
void stopPickup();
// drops the clamp
void dropClamp();
// raises the clamp
void raiseClamp();
// drops the balls
void dropBall();
// resets the ball drop
void resetDrop();

/////////////////////////// Task Declarations ///////////////////////

task t_raiseLiftLow();
task t_raiseLiftMiddle();
task t_raiseLiftHigh();
task t_raiseLiftCenter();
task t_lowerLift();
task t_lowerLiftSlightly();
task t_raiseLiftSlightly();
task t_startPickup();
task t_reversePickup();
task t_stopPickup();
task t_dropClamp();
task t_raiseClamp();
task t_dropBall();
task t_resetDrop();

/////////////////////////// Function Definitions ///////////////////////

// raises the lift
void raiseLift(int seconds) {
	motor[liftMotor] = 100;
	wait1Msec(seconds);
	motor[liftMotor] = 0;
}

// raises the lift
void raiseLift(bool isLift) {
	while(isLift) {
		motor[liftMotor] = 100;
	}
	motor[liftMotor] = 0;
	wait1Msec(1);
}

// lowers the lift
void lowerLift(int r_position) {
	isLift = true;
	motor[liftMotor] = dropPower;
	switch(r_position) {
		case upLow:
			wait1Msec(timeDropGoal1);
			break;
		case upMiddle:
			wait1Msec(timeDropGoal2);
			break;
		case upHigh:
			wait1Msec(timeDropGoal3);
			break;
		case upCenter:
			wait1Msec(timeDropGoal4);
			break;
		default:
			break;
	}
	motor[liftMotor] = 0;
	isLift = false;
}

// lowers the lift
void lowerLift(bool isLift) {
	while(isLift) {
		motor[liftMotor] = -100;
	}
	if(!isLift) {
		motor[liftMotor] = 0;
	}
	wait1Msec(1);
}

// picks up balls
void startPickup() {
	while(true) {
		motor[firstPickupMotor] = 50;
		motor[secondPickupMotor] = 100;
		isPickup = true;
	}
	wait1Msec(1);
}

// releases balls from the pickup
void reversePickup() {
	while(true) {
		motor[firstPickupMotor] = -50;
		motor[secondPickupMotor] = -100;
		isPickup = true;
	}
	wait1Msec(1);
}

// stops the pickup
void stopPickup() {
	motor[firstPickupMotor] = 0;
	motor[secondPickupMotor] = 0;
	isPickup = false;
	wait1Msec(1);
}

// drops the clamp
void dropClamp() {
	servo[clampServo] = 120;
	wait1Msec(1);
}

// raises the clamp
void raiseClamp() {
	servo[clampServo] = 0;
	wait1Msec(1);
}

// drops the balls
void dropBall() {
	servo[dropServo] = 55;
}

// resets the drop servo
void resetDrop() {
	servo[dropServo] = 15;
}

/////////////////////////////// Task Definitions ////////////////////

// raise lift to lowest goal task
task t_raiseLiftLow() {
	raiseLift(timeRaiseGoal1);
	wait1Msec(1);
}

// raise lift to middle goal task
task t_raiseLiftMiddle() {
	raiseLift(timeRaiseGoal2);
	wait1Msec(1);
}

// raise lift to high goal task
task t_raiseLiftHigh() {
	raiseLift(timeRaiseGoal3);
	wait1Msec(1);
}

// raise lift to center goal task
task t_raiseLiftCenter() {
	raiseLift(timeRaiseGoal4);
	wait1Msec(1);
}

// lowers the lift task
task t_lowerLift() {
	lowerLift(position);
	wait1Msec(1);
}

// raises the lift slightly
task t_lowerLiftSlightly() {
	lowerLift(isLift);
	wait1Msec(1);
}

// raises lift slightly
task t_raiseLiftSlightly() {
	raiseLift(isLift);
	wait1Msec(1);
}

// pick up balls
task t_startPickup() {
	startPickup();
	wait1Msec(1);
}

// reverese the pickup
task t_reversePickup() {
	reversePickup();
	wait1Msec(1);
}

task t_stopPickup() {
	stopPickup();
	wait1Msec(1);
}

// drops the clamp
task t_dropClamp() {
	dropClamp();
	wait1Msec(1);
}

// raises the clamp
task t_raiseClamp() {
	raiseClamp();
	wait1Msec(1);
}

// drops the balls
task t_dropBall() {
	dropBall();
	wait1Msec(1);
}

// resets the drop servo
task t_resetDrop() {
	resetDrop();
	wait1Msec(1);
}
