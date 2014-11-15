#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     pickupMotor,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     Blah,   tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     liftMotor,     tmotorTetrix, openLoop, reversed)
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
typedef enum Position {
	upLow,
	upMiddle,
	upHigh,
	upCenter,
	down
};
Position position = down;

// raise lift to lowest goal task
task raise_LiftLow() {
	isRaising = true;
	raiseLift(timeGoal1);
	isRaising = false;
	wait1Msec(1);
}

// raise lift to middle goal task
task raise_LiftMiddle() {
	isRaising = true;
	raiseLift(timeGoal2);
	isRaising = false;
	wait1Msec(1);
}

// raise lift to high goal task
task raise_LiftHigh() {
	isRaising = true;
	raiseLift(timeGoal3);
	isRaising = false;
	wait1Msec(1);
}

// raise lift to center goal task
task raise_LiftCenter() {
	isRaising = true;
	raiseLift(timeGoal4);
	isRaising = false;
	wait1Msec(1);
}

// lowers the lift task
task lower_Lift() {
	isLowering = true;
	lowerLift(position);
	isLowering = false;
	wait1Msec(1);
}

// raises the lift slightly
task lower_Lift_Slightly() {
	lowerLift(s_isLowering);
	wait1Msec(1);
}

// raises lift slightly
task raise_Lift_Slightly() {
	raiseLift(s_isRaising);
	wait1Msec(1);
}

// pick up balls
task pickup() {
	startPickup(isPicking);
	wait1Msec(1);
}

// reverese the pickup
task reverse() {
	reversePickup(isReversing);
	wait1Msec(1);
}

// main function
task main() {
	initializeGlobalVariables();
	Task_Spawn(pickup);
	Task_Spawn(reverse);
	Task_Spawn(raise_Lift_Slightly);
	Task_Spawn(lower_Lift_Slightly);
	while (true) {
		Joystick_UpdateData();

		// driving segment
/**////////////////////////////////////////////////////////////////////////////////////
/**/	// controls the left wheel													///
/**/	motor[leftWheel] = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1); ///
/**/	// controls the right wheel													///
/**/	motor[rightWheel] = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1);///
/**////////////////////////////////////////////////////////////////////////////////////

		// loop controlling the lift
/**////////////////////////////////////////////////////////////////////////////////////////////
/**/	// for the lowest rolling goal														///
/**/	if(Joystick_ButtonReleased(BUTTON_A, CONTROLLER_1) && position == down) {   		///
/**/		Task_Spawn(raise_LiftLow);														///
/**/		position = upLow;																///
/**/	}																					///
/**/	// for the middle rolling goal														///
/**/	else if(Joystick_ButtonReleased(BUTTON_B, CONTROLLER_1) && position == down) { 		///
/**/		Task_Spawn(raise_LiftMiddle);													///
/**/		position = upMiddle;															///
/**/	}																					///
/**/	// for the highest rolling goal														///
/**/	else if(Joystick_ButtonReleased(BUTTON_Y, CONTROLLER_1) && position == down) {		///
/**/		Task_Spawn(raise_LiftHigh);														///
/**/		position = upHigh;																///
/**/	}																					///
/**/	// for the center goal																///
/**/	else if(Joystick_ButtonReleased(BUTTON_X, CONTROLLER_1) && position == down) {		///
/**/		Task_Spawn(raise_LiftCenter);													///
/**/ 		position = upCenter;															///
/**/	}																					///
/**/	// lowers the lift all the way to the ground										///
/**/ 	else if(Joystick_ButtonReleased(BUTTON_START, CONTROLLER_1) && position != down) {	///
/**/		Task_Spawn(lower_Lift);															///
/**/		position = down;																///
/**/	}																					///
/**/	// raises the lift slightly															///
/**/	if(!isRaising && !isLowering) {														///
/**/		while(Joystick_Button(BUTTON_RB, CONTROLLER_1)) 								///
/**/			isRaising = true;															///
/**/		isRaising = false;																///
/**/ 	// lowers the lift slightly															///
/**/		while(Joystick_Button(BUTTON_LB, CONTROLLER_1)) 								///
/**/			isLowering = true;															///
/**/ 		isLowering = false;																///
/**/	}																					///
///////////////////////////////////////////////////////////////////////////////////////////////

		// loop controlling the pickup
/**////////////////////////////////////////////////////////////////////////////////////
/**/	// for picking up 															///
/**/	while(Joystick_Button(BUTTON_RT, CONTROLLER_1))		 						///
/**/		isPicking = true;														///
/**/	isPicking = false;															///
/**/	// for releasing															///
/**/	while(Joystick_Button(BUTTON_LT, CONTROLLER_1)) 							///
/**/		isReversing = true;														///
/**/	isReversing = false;														///
/**////////////////////////////////////////////////////////////////////////////////////
	}
	wait1Msec(1);
}

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
	}
	motor[pickupMotor] = 0;
	wait1Msec(1);
}

// releases balls from the pickup
void reversePickup(bool isReversing) {
	while(isReversing) {
		motor[pickupMotor] = -87;
	}
	motor[pickupMotor] = 0;
	wait1Msec(1);
}
