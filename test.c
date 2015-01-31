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
#include "autoFunctions.h"


task main() {
	servoPrep();
	Joystick_WaitForStart();
	disableDiagnosticsDisplay();
	while(true) {
		encoderPrep();
		startTrackers();
		driveForward(300.0);
		turnLeft(90.0);
		driveForward(200.0);
		//raiseLift(goalPosCenter);
		//dropBallCenter();
		//resetDropCenter();
		//raiseIR();
		//lowerLift(goalPosCenter);
		//dropClamp();

		/*
		// driveForward(300.0);
		// turnRight(90.0);
		// ir loop
		// raiseIR();
		if(!irDetected) {
			// search for ir pos 3
			driveForward(250.0);
			if(irDetected == true) {
				irPos3 = true;
				break;
			}
			// second pos
			if(!irPos1 && !irPos2 && !irPos3) {
				// search for ir pos 1
				// driveForward(250.0);
				// turnLeft(90.0);
				// driveForward(300.0);
				if(irDetected == true) {
					irPos1 = true;
					break;
				}
			}
			// third pos
			if(!irPos1 && !irPos2 && !irPos3) {
				// search for ir pos 2
				// driveForward(100.0);
				// turnLeft(45.0);
				// driveForward(250.0);
				if(irDetected == true) {
					irPos2 = true;
					break;
				}
			}
		}
		lowerIR();
		raiseLift(50);
		// dropBallCenter();
		// resetDropCenter();
		//lowerLift(50);
		if(irPos3) {
			// to high goal
			// driveForward(250.0);
			// turnLeft(90.0);
			// driveForward(800.0)
			// should end up in tile adjacent to high goal
		}
		else if(irPos1) {
			// to high goal
			// driveForward(500.0);
		}
		else if(irPos2) {
			// to high goal
			// turnLeft(90.0);
			// driveBackward(600.0);
		}
		else {
			// finish, autonomous failed
		}
		if(irPos1 || irPos3)	{
		// turnLeft(135.0);
		}
		// driveBackward(200.0);
		// dropClamp();
		// driveFoward(200.0);
		// raiseLift(goalPosHigh);
		// dropBallGoal();
		// lowerLift(goalPosHigh);
		// turnLeft(45.0);
		// driveBackward(800.0);
		// turnRight(90.0);
		// driveBackward(300.0);
		*/
		break;
	}
}
