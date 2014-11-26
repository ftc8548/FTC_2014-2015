#pragma config(Hubs,   S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     firstPickupMotor,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    tmotorTetrix, openLoop,  encoder)
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

///////////////////////// Changable Variables //////////////////////////

const int endIRPos = 120;
const int startIRPos = 0;
const int liftPower = 60;
const int dropPower = -50;
const int pickupPower = 35;
const int stopPower = 0;
const int startPosClamp = 0;
const int endPosClamp = 120;
const int startPosDrop = 15;
const int endPosDrop = 55;
const float wheelSize = 14.6; // in cm
const float circumference = wheelSize * PI;
const float gearRatio = 2;
const float fullPower = 30;
const int timeDropGoal = 1.5 * 1000;
const int timeDropCenter = 3.5 * 1000;
const int timeRaiseGoal = 2 * 1000;
const int timeRaiseCenter = 4 * 1000;
const int timeOneBall = 100;
const int timeFiveBall = 2 * 1000;
const int positionOneBall = 50;
const int positionFiveBall = 255;

/////////////////////// Don't change these variables ///////////////////
float g_vel_curr = 0.0;
float d_vel_curr = 0.0;
float g_vel_prev = 0.0;
float g_dt = 0.0;
float d_dt = 0.0;
float orientation = 0.0;
float error;
float distanceTraveled = 0.0;
bool irDetected = false;
int timer_gyro = 0;
int timer_distance = 0;
typedef enum Position {
	goal,
	center,
	down
};
Position position = down;

///////////////////////////// Function Declarations //////////////////////

// turns the robot left
void turnLeft(float degrees);
// turns the robot right
void turnRight(float degrees);
// moves the robot forward
void driveForward(float distance);
// moves the robot backward
void driveBackward(float distance);
// checks the ir sensor for the beacon
void checkIR();
// starts the pickup
void startPickup();
// reverses the pickup
void reversePickup();
// stops the pickup
void stopPickup();
// raises the lift to the goal
void raiseLift(int seconds);
// lowers the lift
void lowerLift(int seconds);
// drops the balls
void dropBall(int position, int seconds);

/////////////////////////// Task Declarations ///////////////////////////////

// starts the gyro
task a_gyro();
// PID!!!!!
task a_findDistance();
// rasises the ir sensor
task a_raiseIR();
// lowers the ir sensor
task a_lowerIR();
// checks to see if the ir detects the beacon
task a_readIR();
// drops the clamp for roaling goals
task a_dropClamp();
// starts the pickup
task a_startPickup();
// reverses the pickup
task a_reversePickup();
// stops the pickup
task a_stopPickup();
// raises the lift
task a_raiseLiftGoal();
// raises the lift to the center
task a_raiseLiftCenter();
// lowers the lift
task a_lowerLift();
// drops 1 ball
task a_dropOneBall();
// drops 5 balls
task a_dropFiveBall();

///////////////////////////// Function Definitions ///////////////////////////

// turns the robot to the left
void turnLeft(float degrees) {
	bool isTurning = true;
	float target = orientation + degrees;
	float power, power_neg;
	float kP = 3.0;
	float finish_timer = 0.0;

	while(isTurning) {
		error = target - orientation;
		if(abs(error) < 2.5) {
				isTurning = false;
		}
		if(error > 40)
			power = fullPower;
		else if(error < 40)
			power = -fullPower;
		else
			power = kP * error;
		if(abs(power) < 20) {
			if(power > 0)
				power = 20;
			else if(power < 0)
				power = -20;
		}
		power_neg = -power;
		motor[leftWheel] = power_neg;
		motor[rightWheel] = power;
	}
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
}

// turns the robot to the right
void turnRight(float degrees) {
	turnLeft(-degrees);
}

// drives the robot forward
void driveForward(float distance) {
    float target = distanceTraveled + (distance / circumference / gearRatio / 1440);
    bool isMoving = true;
    int power;
    while(isMoving) {
        error = target - distanceTraveled;
        if(error > 500) {
            power = fullPower;
        }
        else if(error < -500) {
            power = -fullPower;
        }
        else if(error > 150) {
            power = 15;
        }
        else if(error < -150) {
            power = -15;
        }
        motor[leftWheel] = power;
        motor[rightWheel] = power;
        if(abs(error) < 150) {
            motor[leftWheel] = 0;
            motor[rightWheel] = 0;
            isMoving = false;
        }
    }
    wait1Msec(1);
}

// moves the robot forward
void driveBackward(float distance) {
	driveForward(-distance);
}

// checks the ir sensor for the beacon
void checkIR() {
	if(SensorValue[irSensor] == 0)
		irDetected = true;
	else
		irDetected = false;
	wait1Msec(1);
}

// starts the pickup
void startPickup() {
	while(true) {
		motor[firstPickupMotor] = pickupPower;
		motor[secondPickupMotor] = pickupPower;
	}
	wait1Msec(1);
}

// reverses the pickup
void reversePickup() {
	while(true) {
		motor[firstPickupMotor] = -pickupPower;
		motor[secondPickupMotor] = -pickupPower;
	}
	wait1Msec(1);
}

// stops the pickup
void stopPickup() {
	motor[firstPickupMotor] = stopPower;
	motor[secondPickupMotor] = stopPower;
	wait1Msec(1);
}

// raises the lift
void raiseLift(int seconds) {
	motor[liftMotor] = liftPower;
	wait1Msec(seconds);
	motor[liftMotor] = stopPower;
}

// lowers the lift
void lowerLift(int seconds) {
	motor[liftMotor] = dropPower;
	wait1Msec(seconds);
	motor[liftMotor] = stopPower;
}

// drops one ball
void dropBall(int position, int seconds) {
	servo[dropServo] = endPosDrop;
	wait1Msec(seconds);
	servo[dropServo] = startPosDrop;
}

////////////////////////////// Task Definitions ///////////////////////////

// starts the gyro
task a_gyro() {
	Time_ClearTimer(timer_gyro);
	while (true) {
		g_vel_prev = g_vel_curr;
		g_dt = (float)Time_GetTime(timer_gyro) / 1000;
		Time_ClearTimer(timer_gyro);
		g_vel_curr = (float)HTGYROreadRot(gyroSensor);
		//orientation += (g_vel_prev + g_vel_curr) * 0.5 * g_dt;
		orientation += g_vel_curr * g_dt;
		wait1Msec(1);
	}
}

// Distance Travelled!
task a_findDistance() {
    Time_ClearTimer(timer_distance);

    while(true) {
        d_dt = (float)Time_GetTime(timer_distance) / 1000.0;
        Time_ClearTimer(timer_distance);
        d_vel_curr = (float)(Motor_GetEncoder(leftWheel) + Motor_GetEncoder(rightWheel)) / 2;
        distanceTraveled += d_vel_curr * d_dt;
        wait1Msec(1);
    }
}

// raises the ir sensor
task a_raiseIR() {
	servo[irServo] = endIRPos;
	wait1Msec(1);
}

// lowers the ir sensor
task a_lowerIR() {
	servo[irServo] = startIRPos;
	wait1Msec(1);
}

// checks the ir for the beacon
task a_readIR() {
	while(true) {
		checkIR();
	}
	wait1Msec(1);
}

// drops the clamp
task a_dropClamp() {
	servo[clampServo] = 120;
	wait1Msec(1);
}

// starts the pickup
task a_startPickup() {
	startPickup();
	wait1Msec(1);
}

// reverses the pickup
task a_reversePickup() {
	reversePickup();
	wait1Msec(1);
}

// stops the pickup
task a_stopPickup() {
	stopPickup();
	wait1Msec(1);
}

// raises the lift
task a_raiseLiftGoal() {
	raiseLift(timeRaiseGoal);
	position = goal;
	wait1Msec(1);
}

// raises the lift to the center
task a_raiseLiftCenter() {
	raiseLift(timeRaiseCenter);
	position = center;
	wait1Msec(1);
}

// lowers the lift
task a_lowerLift() {
	if(position == goal) {
		lowerLift(timeDropGoal);
	}
	if(position == center) {
		lowerLift(timeDropCenter);
	}
	position = down;
	wait1Msec(1);
}

// drops one ball
task a_dropOneBall() {
	dropBall(positionOneBall, timeOneBall);
	wait1Msec(1);
}

// drops five ball
task a_dropFiveBall() {
	dropBall(positionFiveBall, timeFiveBall);
	wait1Msec(1);
}
