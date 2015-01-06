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
#include "teleopFunctions.h"

//										So here are all the button commmands
//													CONTROLLER 1
//					Left Joystick moves the left wheel, Right Joystick moves the right wheel
//					Right Trigger picks up balls, Left Trigger Releases balls
//													CONTROLLER 2
//							Right Bumper raises the lift, Left Bumper lowers the lift
//									Button A drops clamp, Button B raises clamp
//									Button X drops balls, Button Y resets drop

task main() {
	initializeGlobalVariables();
	servoPrep();
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();
		// driving segment

		// the Left Joystick on Controller 1 controls the left wheel
		motor[leftWheel] = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
		// the Right Joystick on Controller 1 controls the right wheel
		motor[rightWheel] = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1);

		// for lift
		// Holding RB on Controller 2 raises the lift
		if(Joystick_Button(BUTTON_RB, CONTROLLER_2)) {
			isLift = true;
			Task_Spawn(t_raiseLift);
		}
		// Releasing RB on Controller 2 stops the lift
		else if(Joystick_ButtonReleased(BUTTON_RB, CONTROLLER_2)) {
			isLift = false;
			Task_Kill(t_raiseLift);
			Task_Spawn(t_stopLift);
		}
		// Holding LB on Controller 2 lowers the lift
		else if(Joystick_Button(BUTTON_LB, CONTROLLER_2)) {
			isLift = true;
			Task_Spawn(t_lowerLift);
		}
		// Releasing LB on Controller 2 stops the lift
		else if(Joystick_ButtonReleased(BUTTON_LB, CONTROLLER_2)) {
			isLift = false;
			Task_Kill(t_lowerLift);
			Task_Spawn(t_stopLift);
		}
		// pressing A on Controller 1 puts lift in low goal pos
		else if(Joystick_Button(BUTTON_A, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_setLiftLow);
		}
		// pressing B on Controller 1 puts lift in middle goal pos
		else if(Joystick_Button(BUTTON_B, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_setLiftMiddle);
		}
		// pressing Y on Controller 1 puts lift in high goal pos
		else if(Joystick_Button(BUTTON_Y, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_setLiftHigh);
		}
		// pressing X on Controller 1 puts lift in center goal pos
		else if(Joystick_Button(BUTTON_X, CONTROLLER_1) && !isLift) {
			Task_Spawn(t_setLiftCenter);
		}

		// loop controlling the pickup
		// Holding RT on Controller 1 picks up balls
		if(Joystick_Button(BUTTON_RT, CONTROLLER_1)) {
			Task_Kill(t_reversePickup);
			Task_Spawn(t_startPickup);
		}
		// Releasing RT on Controller 1 stops the pickup
		else if(Joystick_ButtonReleased(BUTTON_RT, CONTROLLER_1)) {
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
