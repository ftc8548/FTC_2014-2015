#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CCustom)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor, tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     rightWheel,    tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     liftMotor,     tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    dropServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    clampServoR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    clampServoL,          tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    irServo,              tServoStandard)

#include "includes.h"
#include "autoFunctions.h"

task main() {
	servoPrep();
	disableDiagnosticsDisplay();
	Joystick_WaitForStart();
	startTrackers();
	while(true) {
		while (orientation < 90.0 && orientation > - 90.0) {
			//motor[rightWheel] = 60;
			//motor[leftWheel] = -60;
		}
		motor[rightWheel] = 0;
		motor[leftWheel] = 0;
		//turnRight(90.0);
		break;
	}
}
