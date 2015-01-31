#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     gyroSensor,     sensorAnalogInactive)
#pragma config(Sensor, S3,     	 irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     				tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor, 		tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     rightWheel,    				tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     liftMotor,     				tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    centerServo,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    goalServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    clampServoR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    clampServo,          	tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    irServo,              tServoStandard)
//#pragma config(Servo,	srvo_S1_C2_5,		clampServoL,						tServoStandard)
#include "includes.h"
#include "teleopFunctions.h"

float goalPos = 0.0;

//										So here are all the button commmands
//													CONTROLLER 1
//							Start puts robot in full power, Back puts robot in fine tuning
//					Left Joystick moves the left wheel, Right Joystick moves the right wheel
//								Right Trigger picks up balls, Left Trigger Releases balls
//							Right Bumper raises the lift, Left Bumper lowers the lift
//													CONTROLLER 2
//									Button A drops clamp, Button B raises clamp
//									Button X drops balls, Button Y resets drop

task main() {
	initializeGlobalVariables();
	servoPrep();
	Task_Spawn(t_liftEncoder);
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();
		nxtDisplayTextLine(1, "lift: %d", l_distanceTraveled);
		// driving segment

		// for full power, press start controller 1
		if (Joystick_Button(BUTTON_START)) {
			isFullPower = true;
		}
		// for fine tuning, press back on controller 1
		else if (Joystick_Button(BUTTON_BACK)) {
			isFullPower = false;
		}

		// full power segment
		if(isFullPower) {
			drivePowerL = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
			drivePowerR = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		}
		// fine tuning section
		else {
			drivePowerL = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1) / 2;
			drivePowerR = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1) / 2;
		}

			// the Left Joystick on Controller 1 controls the left wheel
			motor[leftWheel] = drivePowerL;
			// the Right Joystick on Controller 1 controls the right wheel
			motor[rightWheel] = drivePowerR;

		// for lift
		// Holding RB on Controller 1 raises the lift
		if(Joystick_Button(BUTTON_RB, CONTROLLER_1))	{
			isLift = true;
			Task_Spawn(t_raiseLift);
		}
		// Releasing RB on Controller 1 stops the lift
		else if(Joystick_ButtonReleased(BUTTON_RB, CONTROLLER_1)) {
			isLift = false;
			Task_Kill(t_raiseLift);
			Task_Spawn(t_stopLift);
		}
		// Holding LB on Controller 1 lowers the lift
		else if(Joystick_Button(BUTTON_LB, CONTROLLER_1))	{
			isLift = true;
			Task_Spawn(t_lowerLift);
		}
		// Releasing LB on Controller 1 stops the lift
		else if(Joystick_ButtonReleased(BUTTON_LB, CONTROLLER_1)) {
			isLift = false;
			Task_Kill(t_lowerLift);
			Task_Spawn(t_stopLift);
		}
		// pressing B on Controller 1 puts lift in low goal pos
		/*
		else if(Joystick_Button(BUTTON_B, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_raiseLiftLow);
			goalPos = goalPosLow;
		}
		 pressing X on Controller 1 puts lift in middle goal pos
		else if(Joystick_Button(BUTTON_X, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_raiseLiftMiddle);
			goalPos = goalPosMid;
		}
		 // pressing Y on Controller 1 puts lift in high goal pos
		else if(Joystick_Button(BUTTON_Y, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_raiseLiftHigh);
			goalPos = goalPosHigh;
		}
		// pressing A on Controller 1 lowers lift to ground
		else if(Joystick_Button(BUTTON_A, CONTROLLER_1) && !isLift && goalPos == goalPosLow) {
			Task_Spawn(t_lowerLiftLow);
		}
		else if(Joystick_Button(BUTTON_A, CONTROLLER_1) && !isLift && goalPos == goalPosMid) {
			Task_Spawn(t_lowerLiftMiddle);
		}
		else if(Joystick_Button(BUTTON_A,CONTROLLER_1) && !isLift && goalPos == goalPosHigh) {
			Task_Spawn(t_lowerLiftHigh);
		}
		// make a kill lift button
		*/

		// loop controlling the pickup
		// Holding RT on Controller 1 picks up balls
		if(Joystick_Button(BUTTON_RT, CONTROLLER_1)) {
			Task_Kill(t_reversePickup);
			Task_Spawn(t_startPickup);
		}
		// Releasing RT on Controller 1 stops the pickup
		 if(Joystick_ButtonReleased(BUTTON_RT, CONTROLLER_1)) {
			Task_Kill(t_startPickup);
			Task_Spawn(t_stopPickup);
		}
		// Holding LT on Controller 1 releases balls from the pickup
		else if(Joystick_Button(BUTTON_LT, CONTROLLER_1)) {
			Task_Kill(t_startPickup);
			Task_Spawn(t_reversePickup);
		}
		// Releasing LT on Controller 1 stops the pickup
		else if(Joystick_ButtonReleased(BUTTON_LT, CONTROLLER_1)) {
			Task_Kill(t_reversePickup);
			Task_Spawn(t_stopPickup);
		}

		// for clamp
		// Pressing Button A on Controller 2 drops the clamp
		if(Joystick_Button(BUTTON_A, CONTROLLER_2)) {
			Task_Kill(t_raiseClamp);
			Task_Spawn(t_dropClamp);
		}
		// Pressing Button B on Controller 2 raises the clamp
		if(Joystick_Button(BUTTON_B, CONTROLLER_2)) {
			Task_Kill(t_dropClamp);
			Task_Spawn(t_raiseClamp);
		}

		// for the drop servo
		// Pressing X on Controller 2 drops the balls from the basket
		if(Joystick_Button(BUTTON_X, CONTROLLER_2)) {
			Task_Kill(t_resetDrop);
			Task_Spawn(t_dropBall);
		}
		// Pressing Y on Controller 2 resets the drop servo so balls doon't fall out of the basket
		if(Joystick_Button(BUTTON_Y, CONTROLLER_2)) {
			Task_Kill(t_dropBall);
			Task_Spawn(t_resetDrop);
		}
		wait1Msec(1);
	}
}
