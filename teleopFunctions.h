#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     pickupMotor,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     Blah,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_1,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    IRMotor,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"

// raises the lift
void raiseLift(int seconds);
void raiseLift(bool isRaising);
// lowers the lift
void lowerLift(int r_position);
void lowerLift(bool isLowering);
// starts the pickup
void startPickup(bool isPicking);
// reverses the pickup
void reversePickup(bool isReversing);
// stops the pickup
void stopPickup(bool isTurning);

task t_raiseLiftLow();
task t_raiseLiftMiddle();
task t_raiseLiftHigh();
task t_raiseLiftCenter();
task t_lowerLift();
task t_lowerLiftSlightly();
task t_raiseLiftSlightly();
task t_startPickup();
task t_stopPickup();
task t_reversePickup();


const int timeGoal1 = 2 *1000;
const int timeGoal2 = 4 *1000;
const int timeGoal3 = 6 *1000;
const int timeGoal4 = 8 *1000;
bool s_isLowering = false;
bool s_isRaising = false;
bool isLowering = false;
bool isRaising = false;
bool isPicking = false;
bool isReversing = false;
bool isTurning = false;

typedef enum Position {
	upLow,
	upMiddle,
	upHigh,
	upCenter,
	down
};
Position position = down;

// raises the lift
void raiseLift(int seconds) {
	motor[liftMotor] = 87;
	wait1Msec(seconds);
	motor[liftMotor] = 0;
}

// raises the lift
void raiseLift(bool isRaising) {
	motor[liftMotor] = 50;
	while(isRaising) {
		wait1Msec(1);
	}
	motor[liftMotor] = 0;
}

// lowers the lift
void lowerLift(int r_position) {
	motor[liftMotor] = -87;
	switch(r_position) {
		case upLow:
			wait1Msec(timeGoal1);
			break;
		case upMiddle:
			wait1Msec(timeGoal2);
			break;
		case upHigh:
			wait1Msec(timeGoal3);
			break;
		case upCenter:
			wait1Msec(timeGoal4);
			break;
		default:
			break;
	}
	motor[liftMotor] = 0;
}

// lowers the lift
void lowerLift(bool isLowering) {
	motor[liftMotor] = -50;
	while(isLowering) {
		wait1Msec(1);
	}
	motor[liftMotor] = 0;
}

// picks up balls
void startPickup(bool isPicking) {
	while(isPicking) {
		motor[pickupMotor] = 87;
		wait1Msec(100);
	}
	wait1Msec(1);
}

// releases balls from the pickup
void reversePickup(bool isReversing) {
	while(isReversing) {
		motor[pickupMotor] = -87;
		wait1Msec(100);
	}
	wait1Msec(1);
}

// stops the pickup
void stopPickup(bool isTurning) {
	while(isTurning) {
		motor[pickupMotor] = 0;
		wait1Msec(1);
	}
	wait1Msec(1);
}

// raise lift to lowest goal task
task t_raiseLiftLow() {
	isRaising = true;
	raiseLift(timeGoal1);
	isRaising = false;
	wait1Msec(1);
}

// raise lift to middle goal task
task t_raiseLiftMiddle() {
	isRaising = true;
	raiseLift(timeGoal2);
	isRaising = false;
	wait1Msec(1);
}

// raise lift to high goal task
task t_raiseLiftHigh() {
	isRaising = true;
	raiseLift(timeGoal3);
	isRaising = false;
	wait1Msec(1);
}

// raise lift to center goal task
task t_raiseLiftCenter() {
	isRaising = true;
	raiseLift(timeGoal4);
	isRaising = false;
	wait1Msec(1);
}

// lowers the lift task
task t_lowerLift() {
	isLowering = true;
	lowerLift(position);
	isLowering = false;
	wait1Msec(1);
}

// raises the lift slightly
task t_lowerLiftSlightly() {
	lowerLift(s_isLowering);
	wait1Msec(1);
}

// raises lift slightly
task t_raiseLiftSlightly() {
	raiseLift(s_isRaising);
	wait1Msec(1);
}

// pick up balls
task t_startPickup() {
	startPickup(isPicking);
	wait1Msec(1);
}

// reverese the pickup
task t_reversePickup() {
	reversePickup(isReversing);
	wait1Msec(1);
}

task t_stopPickup() {
	stopPickup(isTurning);
	wait1Msec(1);
}
