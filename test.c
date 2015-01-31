#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     gyroSensor,     sensorAnalogInactive)
#pragma config(Sensor, S3,     	 irSensor,     sensorI2CCustom9V)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     				tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor, 		tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     rightWheel,    				tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     liftMotor,     				tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    goalServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    centerServo,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    clampServo,          	tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    irServo,              tServoStandard)

#include "includes.h"
#include "autoFunctions.h"


task main() {
	servoPrep();
	Joystick_WaitForStart();
	disableDiagnosticsDisplay();
	while(true) {
		encoderPrep();
		startTrackers();
		driveForward(310.0);
		turnRight(85.0);
		if(irDetected == true) {
			irPos2 = true;
			turnRight(40.0);
			irDetected = false;
			driveBackward(25.0);
			if(irDetected == true) {
				driveBackward(15.0);
				raiseLift(750.0);
				dropBallCenter();
				wait1Msec(300);
				servo[centerServo] = endPosCenter - 30;
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				driveForward(105.0);
				turnRight(85.0);
				driveBackward(300.0);
			}
			else	{
				irPos1 = true;
				driveBackward(110.0);
				turnRight(46.0);
				driveBackward(135.0);
				turnRight(3.0);
				raiseLift(750.0);
				dropBallCenter();
				wait1Msec(300);
				servo[centerServo] = endPosCenter - 30;
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				driveForward(130.0);
				turnRight(82.0);
				driveBackward(300.0);
			}
		}
		else	{
			driveForward(115.0);
			// ir loop
			if(irDetected == true) {
				irPos3 = true;
				driveBackward(15.0);
				raiseLift(750.0);
				dropBallCenter();
				wait1Msec(300);
				servo[centerServo] = endPosCenter - 30;
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				lowerLift(1000.0);
				driveForward(120.0);
				turnRight(85.0);
				driveBackward(300.0, 60);
				/*turnRight(43.0);
				driveBackward(250.0);
				motor[rightWheel] = -20;
				motor[leftWheel] = -20;
				wait1Msec(200);
				motor[rightWheel] = 0;
				motor[leftWheel] = 0;
				dropClamp();
				driveForward(10.0);
				raiseLift(700.0);
				dropBallGoal();
				resetDropGoal();*/
				break;
			}
			else	{
				driveBackward(220.0);
				turnRight(85.0);
				driveBackward(100.0);
			}
		}
		break;
	}
}
