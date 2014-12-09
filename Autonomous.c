#pragma config(Hubs,   S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,    			tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    			tmotorTetrix, openLoop,	reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     liftMotor,          tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C4_1, 		secondPickupMotor,	tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     liftMotor,     			tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    irServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    clampServo,         tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    dropServo,          tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,             tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,             tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,             tServoNone)
//#pragma config(Motor,  mtr_S1_C4_1,     liftMotor,     tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"
#include "autoFunctions.h"

typedef enum GoalPosition {
	goalPos1,
	goalPos2,
	goalPos3
};
GoalPosition goalPosition;

task main() {
	Joystick_WaitForStart();

	Task_Spawn(a_gyro);
	Task_Spawn(a_wheeEncoder);
	Task_Spawn(a_liftEncoder);
	while(orientation < 90) {
		motor[rightWheel] = 100;
		motor[leftWheel] = -100;
	}
	motor[rightWheel] = 0;
	motor[leftWheel] = 0;
	wait1Msec(2000);
	while(abs(d_distanceTraveled) < 40) {
		motor[rightWheel] = 100;
		motor[leftWheel] = 100;
	}

	//Task_Spawn(a_findDistance);
	//Task_Spawn(a_raiseIR);
	/*Task_Spawn(a_readIR);
	// grab a rolling goal here
	driveBackward(100);
	Task_Spawn(a_dropClamp);
	// do ball drop
	Task_Spawn(a_raiseLiftGoal);
	// drop the balls
	Task_Spawn(a_dropOneBall);
	// lowers the lift
	Task_Spawn(a_lowerLift);
	*/

	// drive to the center
	while(!irDetected) {
		// ir detection stuff goes here
		driveForward(20.0);
		turnLeft(90);
		turnRight(180);
		driveBackward(20.0);
	}
	// enter stuff for after ir detected here
	// also known as putting balls into the rolling goals, knocking down the kickstand
	// so first a bit of driving
	Task_Spawn(a_raiseLiftCenter);
	// drop balls
	Task_Spawn(a_dropOneBall);
	// lower the lift
	Task_Spawn(a_lowerLift);
	// now knock down kickstand
	// after kickstand ball pickup starts
	Task_Spawn(a_startPickup);
	// drive around for a bit
	// do the ball drop and move away before dropping ir
	Task_Spawn(a_lowerIR);
	// drive around for a bit
	Task_Spawn(a_reversePickup);
	// drive for a bit
	Task_Spawn(a_stopPickup);

	Task_Spawn(a_raiseLiftGoal);
	Task_Spawn(a_dropFiveBall);
	Task_Spawn(a_lowerLift);

	driveForward(10.0);
	turnLeft(90.0);
	turnRight(180.0);
	driveForward(20.0);
}
