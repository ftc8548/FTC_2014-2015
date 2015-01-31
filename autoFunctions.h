#include "includes.h"

///////////////////////// Changable Variables //////////////////////////

// servos
const int endIRPos = 125;
const int startIRPos = 35;
const int startPosClamp = 10;
const int endPosClamp = 110;
const int startPosDrop = 187;
const int endPosDrop = 147;
const int startPosCenter = 0;
const int endPosCenter = 110;
// driving powers
const int turnPower = 40;
const int optimalMaxDrivePower = 40;
const int optimalMinDrivePower = 20;
const int maxMaxDrivePower = 100;
const int minMinDrivePower = 10;
const int optimalMaxTurnPower = 40;
const int optimalMinTurnPower = 17;
const int maxMaxTurnPower = 100;
const int minMinDriveTurnPower = 17;
const int minMinTurnPower = 17;
const int rampPower = 25;
const int raisePower = 100;
const int lowerPower = 80;
const int liftPower = 100;
// PID variables
const int andyPulseValue = 280;
const int regPulseValue = 360;
const float d_wheelDiam = 6.985; // in cm
const float l_wheelDiam = 10.0; // in cm
const float d_circumference = d_wheelDiam * PI;
const float l_circumference = l_wheelDiam * PI;
const float d_gearRatio = 2.0;
const float l_gearRatio = 1.0;
const int fineTuneTime = 1000;
// lift variables
const int goalPosHigh = 80;
const int goalPosCenter = 120;
// ir variables
short irA, irB, irC, irD, irE;
bool irPos1 = false;
bool irPos2 = false;
bool irPos3 = false;
// general variables
const int waitTime = 400;

/////////////////////// Don't change these variables ///////////////////

float orientation = 0.0;
float d_distanceTraveled = 0.0;
float d_rightSide = 0.0;
float d_leftSide = 0.0;
float l_distanceTraveled = 0.0;
bool irDetected = false;
float g_vel_prev = 0.0;
float g_vel_curr = 0.0;

///////////////////////////// Function Declarations //////////////////////

// gets the servos ready
void servoPrep();
// gets the encoders ready
void encoderPrep();
// starts the tracker tasks
void startTrackers();
// raises ir
void raiseIR();
// lowers ir
void lowerIR();
// raises clamp servos
void raiseClamp();
// drops clamp servos
void dropClamp();
// drops balls
void dropBallGoal();
// resets drop
void resetDropGoal();
// drops ball into center goal
void dropBallCenter();
// resets center drop
void resetDropCenter();
// drives the robot down the ramp
void ramp(float distance);
// moves the robot forward
void driveForward(float distance, int maxPower = optimalMaxDrivePower, int minPower = optimalMinDrivePower);
// moves the robot backward
void driveBackward(float distance, int maxPower = optimalMaxDrivePower, int minPower = optimalMinDrivePower);
// turns the robot right
void turnRight(float degrees, int maxPower = optimalMaxTurnPower, int minPower = optimalMinTurnPower);
// turns the robot left
void turnLeft(float degrees, int maxPower = optimalMaxTurnPower, int minPower = optimalMinTurnPower);
// raises the lift to the goal
void raiseLift(float distance);
// lowers the lift
void lowerLift(float distance);
// sets the lift position
void setLift(float pos);

/////////////////////////// Task Declarations ///////////////////////////////

// starts the gyro
task gyro();
// tells how far robot has travelled
task wheelEncoder();
// tells where the lift is
task liftEncoder();
// checks to see if the ir detects the beacon
task readIR();
// lowers lift
task t_lowerLift();

///////////////////////////// Function Definitions ///////////////////////////

// gets the servos ready
void servoPrep() {
	raiseClamp();
	resetDropGoal();
	resetDropCenter();
	raiseIR();
}

// gets the encoders ready
void encoderPrep() {
	nMotorEncoder(leftWheel) = 0;
	nMotorEncoder(rightWheel) = 0;
	nMotorEncoder(liftMotor) = 0;
}

// starts the tracker tasks
void startTrackers() {
	Task_Spawn(gyro);
	Task_Spawn(liftEncoder);
	Task_Spawn(wheelEncoder);
	Task_Spawn(readIR);
	wait1Msec(waitTime);
}

// raises the ir sensor
void raiseIR() {
	servo[irServo] = endIRPos;
	wait1Msec(100);
}

// lowers the ir sensor
void lowerIR() {
	servo[irServo] = startIRPos;
	wait1Msec(100);
}

// raises clamp servos
void raiseClamp() {
	servo[clampServo] = startPosClamp;
	wait1Msec(100);
}

// drops clamp servos
void dropClamp() {
	servo[clampServo] = endPosClamp;
	wait1Msec(100);
}

// drops balls into goal
void dropBallGoal() {
	servo[goalServo] = endPosDrop;
	wait1Msec(700);
}

// reset goal drop
void resetDropGoal() {
	servo[goalServo] = startPosDrop;
	wait1Msec(100);
}

// drops ball into center
void dropBallCenter() {
	servo[centerServo] = endPosCenter;
	wait1Msec(500);
}

// resets center drop
void resetDropCenter() {
	servo[centerServo] = startPosCenter;
	wait1Msec(100);
}

void driveForward(float distance, int maxPower, int minPower) {
	bool isMoving = true;
	bool isFineTune = false;
	int timer = 0.0;
	int totalTimer = 0.0;
	int fineTuneTimer = 0.0;
	float l_power, r_power;
	float r_errorPower = 0.0;
	float l_errorPower = 0.0;
	float d_leftTarget = d_leftSide + (distance / d_circumference / d_gearRatio) * andyPulseValue;
	float d_rightTarget = d_rightSide + (distance / d_circumference / d_gearRatio) * andyPulseValue;
	float d_kP = 0.7;
	float d_kI = 0.0;
	float currDt = 0.0;
	float d_leftPIDValue = 0.0;
	float d_rightPIDValue = 0.0;
	float d_leftCurrError = 0.0;
	float d_leftPrevError = 0.0;
	float d_leftErrorRate = 0.0;
	float d_leftAccumError = 0.0;
	float d_rightCurrError = 0.0;
	float d_rightPrevError = 0.0;
	float d_rightErrorRate = 0.0;
	float d_rightAccumError = 0.0;
	float t_target = orientation;
	float t_kP = 3.0;
	float t_kI = 0.0;
	float t_PIDValue = 0.0;
	float t_currError = 0.0;
	float t_prevError = 0.0;
	float t_errorRate = 0.0;
	float t_accumError = 0.0;
	Time_ClearTimer(timer);
	Time_ClearTimer(totalTimer);

	while(isMoving) {
		currDt = Time_GetTime(timer) / 1000;
		Time_ClearTimer(timer);
		d_leftPrevError = d_leftCurrError;
		d_leftCurrError = d_leftTarget - d_leftSide;
		d_leftErrorRate = d_leftCurrError - d_leftPrevError;
		d_leftAccumError += d_leftErrorRate * currDt;
		d_leftPIDValue = d_kP * d_leftCurrError + d_kI * d_leftAccumError;

		d_rightPrevError = d_rightCurrError;
		d_rightCurrError = d_rightTarget - d_rightSide;
		d_rightErrorRate = d_rightCurrError - d_rightPrevError;
		d_rightAccumError += d_rightErrorRate * currDt;
		d_rightPIDValue = d_kP * d_rightCurrError + d_kI * d_rightAccumError;

		t_prevError = t_currError;
		t_currError = t_target - orientation;
		t_errorRate = t_currError - t_prevError;
		t_accumError += t_errorRate * currDt;
		t_PIDValue = t_kP * t_currError + t_kI * t_accumError;

		if(d_leftPIDValue > maxPower) {
			l_power = maxPower;
		}
		else if(d_leftPIDValue < -maxPower) {
			l_power = -maxPower;
		}
		else if(d_leftPIDValue < maxPower && d_leftPIDValue > minPower) {
			l_power = (int)((float)d_leftPIDValue);
		}
		else if(d_leftPIDValue > -maxPower && d_leftPIDValue < -minPower) {
			l_power = (int)((float)d_leftPIDValue);
		}
		else if(d_leftPIDValue < minPower) {
			l_power = minPower;
		}
		else if(d_leftPIDValue > -minPower) {
			l_power = -minPower;
		}

		if(d_rightPIDValue > maxPower) {
			r_power = maxPower;
		}
		else if(d_rightPIDValue < -maxPower) {
			r_power = -maxPower;
		}
		else if(d_rightPIDValue < maxPower && d_rightPIDValue > minPower) {
			r_power = (int)((float)d_leftPIDValue);
		}
		else if(d_leftPIDValue > -maxPower && d_rightPIDValue < -minPower) {
			r_power = (int)((float)d_rightPIDValue);
		}
		else if(d_rightPIDValue < minPower) {
			r_power = minPower;
		}
		else if(d_rightPIDValue < -minPower) {
			r_power = -minPower;
		}

		if(t_PIDValue > 30) {
			l_errorPower = 30;
			r_errorPower = 0;
		}
		else if(t_PIDValue < -30) {
			l_errorPower = 0;
			r_errorPower = 30;
		}
		else if(t_PIDValue < 30 && t_PIDValue > minMinDriveTurnPower) {
			l_errorPower = t_PIDValue;
			r_errorPower = 0;
		}
		else if(t_PIDValue > -30 && t_PIDValue < -minMinDriveTurnPower) {
			l_errorPower = 0;
			r_errorPower = t_PIDValue;
		}
		else if(t_PIDValue < minMinDriveTurnPower && t_PIDValue > 0) {
			if(d_rightSide < d_leftSide) {
				l_errorPower = minMinDriveTurnPower;
				r_errorPower = 0;
			}
			else if(d_rightSide > d_leftSide) {
				l_errorPower = 0;
				r_errorPower = -minMinDriveTurnPower;
			}
		}
		else if(t_PIDValue > -minMinDriveTurnPower && t_PIDValue < 0) {
			if(d_rightSide < d_leftSide) {
				l_errorPower = 0;
				r_errorPower = minMinDriveTurnPower;
			}
			else if(d_rightSide > d_leftSide) {
				l_errorPower = -minMinDriveTurnPower ;
				r_errorPower = 0;
			}
		}

		if(abs(d_leftCurrError) < 50) {
			l_power = 0;
		}
		if(abs(d_rightCurrError) < 50) {
			r_power = 0;
		}
		if(abs(t_currError) < 0.5) {
			r_errorPower = 0;
			l_errorPower = 0;
		}
		if(abs(d_leftCurrError) < 50 && abs(d_rightCurrError) < 50 && abs(t_currError) < 0.5) {
			isMoving = false;
			isFineTune = true;
		}
		motor[leftWheel] = l_power + l_errorPower;
		motor[rightWheel] = r_power + r_errorPower;
		if(Time_GetTime(totalTimer) > 3000) {
			motor[rightWheel] = 0;
			motor[leftWheel] = 0;
			isMoving = false;
			break;
		}
	}
	/*
	if(isFineTune) {
		Time_ClearTimer(fineTuneTimer);
		while(Time_GetTime(fineTuneTimer) < fineTuneTime) {
			if(d_leftTarget - d_leftSide > 0)	{
				l_power = minMinDrivePower;
			}
			else if(d_leftTarget - d_leftSide < 0) {
				l_power = -minMinDrivePower;
			}
			else	{
				l_power = 0;
			}

			if(d_rightTarget - d_rightSide > 0)	{
				r_power = minMinDrivePower;
			}
			else if(d_rightTarget - d_rightSide < 0) {
				r_power = -minMinDrivePower;
			}
			else	{
				r_power = 0;
			}

			if(t_target - orientation > 0)	{
				r_errorPower = 2;
				l_errorPower = 0;
			}
			else if(t_target - orientation < 0) {
				r_errorPower = 0;
				l_errorPower = 2;
			}
			else	{
				r_errorPower = 0;
				l_errorPower = 0;
			}
			motor[leftWheel] = l_power + l_errorPower;
			motor[rightWheel] = r_power + r_errorPower;
		}
	}
	*/
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
	wait1Msec(waitTime);
}

// moves the robot forward
void driveBackward(float distance, int maxPower, int minPower) {
	driveForward(-distance, maxPower, minPower);
}

// turns the robot to the right
void turnRight(float degrees, int maxPower, int minPower) {
	bool isTurning = true;
	bool isFineTune = false;
	int timer = 0.0;
	int totalTimer = 0.0;
	int fineTuneTimer = 0.0;
	float target = orientation + degrees;
	float t_power;
	float kP = 3.0;
	float kI = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float accumError = 0.0;

	Time_ClearTimer(timer);
	Time_ClearTimer(totalTimer);
	while(isTurning) {
		currDt = Time_GetTime(timer) / 1000;
		Time_ClearTimer(timer);
		prevError = currError;
		currError = target - orientation;
		errorRate = prevError - currError;
		accumError += errorRate * currDt;
		PIDValue = kP * currError + kI * accumError;

		if(PIDValue > maxPower)	{
			t_power = maxPower;
		}
		else if(PIDValue < -maxPower)	{
			t_power = -maxPower;
		}
		else if(PIDValue < maxPower && PIDValue > minPower)	{
			t_power = PIDValue;
		}
		else if(PIDValue > -maxPower && PIDValue < -minPower) {
			t_power = PIDValue;
		}
		else if(PIDValue < minPower && PIDValue > 0.0)	{
			t_power = minPower;
		}
		else if(PIDValue > -minPower && PIDValue < 0.0)	{
			t_power = -minPower;
		}
		if(abs(currError) < 1.0) {
			t_power = 0;
			isFineTune = true;
			isTurning = false;
		}
		if(Time_GetTime(totalTimer) > 3000) {
			motor[rightWheel] = 0;
			motor[leftWheel] = 0;
			isTurning = false;
			isFineTune = false;
			break;
		}
		motor[leftWheel] = t_power;
		motor[rightWheel] = -t_power;
	}
	if(isFineTune) {
		Time_ClearTimer(fineTuneTimer);
		while(Time_GetTime(fineTuneTimer) < fineTuneTime) {
			if(target - orientation > 0.3)	{
				t_power = minMinTurnPower;
			}
			else if(target - orientation < -0.3) {
				t_power = -minMinTurnPower;
			}
			else	{
				t_power = 0;
			}
			motor[leftWheel] = t_power;
			motor[rightWheel] = -t_power;
		}
	}
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
	wait1Msec(waitTime);
}

// turns the robot to the left
void turnLeft(float degrees, int maxPower, int minPower) {
	turnRight(-degrees, maxPower, minPower);
}

// raises the lift to the goal
void raiseLift(float distance) {
	float target;
  bool isLifting = true;
  bool isFineTune = false;
  int timer = 0.0;
  int fineTuneTimer = 0.0;
	float l_power;
	float kP = 0.3;
	float kI = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float accumError = 0.0;

	target = l_distanceTraveled + (distance / l_circumference /l_gearRatio) * regPulseValue;
	Time_ClearTimer(timer);
	while(isLifting) {
		currDt = Time_GetTime(timer) / 1000;
		Time_ClearTimer(timer);
		prevError = currError;
		currError = target - l_distanceTraveled;
		errorRate = prevError - currError;
		accumError += errorRate * currDt;
		PIDValue = kP * currError + kI * accumError;

		if(PIDValue > raisePower) {
			l_power = raisePower;
		}
		else if(PIDValue < -lowerPower) {
			l_power = -lowerPower;
		}
		else if(PIDValue < raisePower && PIDValue > 40) {
			l_power = PIDValue;
		}
		if(currError < 50)	{
			l_power = 0;
			isLifting = false;
			isFineTune = true;
		}
		motor[liftMotor] = l_power;
	}
	if(isFineTune) {
		Time_ClearTimer(fineTuneTimer);
		while(Time_GetTime(fineTuneTimer) < fineTuneTime) {
			if(target - l_distanceTraveled > 5) {
				l_power = 20;
			}
			else if(target - l_distanceTraveled < 5) {
				l_power = -10;
			}
			else {
				l_power = 0;
			}
			motor[liftMotor] = l_power;
		}
	}
	motor[liftMotor] = 0;
	wait1Msec(waitTime);
}

// lowers the lift
void lowerLift(float distance) {
	raiseLift(-distance);
}

////////////////////////////// Task Definitions ///////////////////////////

// starts the gyro
task gyro() {
	int timer_gyro = 0;
	g_vel_curr = 0.0;
	float g_dt = 0.0;
	HTGYROstartCal(gyroSensor);
	Time_ClearTimer(timer_gyro);
	while (true) {
		g_dt = (float)Time_GetTime(timer_gyro) / 1000.0;
		Time_ClearTimer(timer_gyro);
		g_vel_curr = (float)HTGYROreadRot(gyroSensor);
		orientation += g_vel_curr * g_dt;
		nxtDisplayTextLine(0, "ang: %d", orientation);
		wait1Msec(1);
	}
}

// Distance Travelled by robot
task wheelEncoder() {
	d_distanceTraveled = 0.0;
	d_leftSide = 0.0;
	d_rightSide = 0.0;
	float tickRight, tickLeft;
  while(true) {
  	d_leftSide = (float) (Motor_GetEncoder(leftWheel));
    d_rightSide = (float) (Motor_GetEncoder(rightWheel));
    d_distanceTraveled = (float) (Motor_GetEncoder(leftWheel) + Motor_GetEncoder(rightWheel)) / 2.0;
    tickLeft = Motor_GetEncoder(leftWheel);
    tickRight = Motor_GetEncoder(rightWheel);
    nxtDisplayTextLine(1, "leftSide: %d", d_leftSide);
    nxtDisplayTextLine(2, "rightSide: %d", d_rightSide);
    wait1Msec(1);
  }
}

// tells position of the lift
task liftEncoder() {
	while(true) {
		l_distanceTraveled = (float) (Motor_GetEncoder(liftMotor));
		wait1Msec(1);
	}
}

// checks the ir for the beacon
task readIR() {
	while(true) {
		HTIRS2setDSPMode(irSensor, DSP_1200);
		HTIRS2readAllACStrength(irSensor, irA, irB, irC, irD, irE);
		// 0 to 50, 0 being no and 50 being super close
		nxtDisplayTextLine(3, "irA: %d", irA);
		nxtDisplayTextLine(4, "irB: %d", irB);
		nxtDisplayTextLine(5, "irC: %d", irC);
		nxtDisplayTextLine(6, "irD: %d", irD);
		nxtDisplayTextLine(7, "irE: %d", irE);
		if(irC >= 50) {
			irDetected = true;
		}
		wait1Msec(1);
	}
}

task t_lowerLift() {
	lowerLift(900.0);
	lowerLift(900.0);
	lowerLift(900.0);
	lowerLift(900.0);
}

/*

// drives the robot forward
void driveForward(float distance) {
	bool isMoving = true;
	bool isFineTune = false;
	int timer = 0.0;
	int fineTuneTimer = 0.0;
	float d_power;
	float r_errorPower = 0.0;
	float l_errorPower = 0.0;
	float d_target = d_distanceTraveled + (distance / d_circumference / d_gearRatio) * andyPulseValue;
	float d_kP = 2.0;
	float d_kI = 0.0;
	float currDt = 0.0;
	float d_PIDValue = 0.0;
	float d_currError = 0.0;
	float d_prevError = 0.0;
	float d_errorRate = 0.0;
	float d_accumError = 0.0;
	float t_target = orientation;
	float t_kP = 0.0;
	float t_kI = 0.0;
	float t_PIDValue = 0.0;
	float t_currError = 0.0;
	float t_prevError = 0.0;
	float t_errorRate = 0.0;
	float t_accumError = 0.0;
	Time_ClearTimer(timer);

	while(isMoving) {
		currDt = Time_GetTime(timer) / 1000;
		Time_ClearTimer(timer);
		d_prevError = d_currError;
		d_currError = d_target - d_distanceTraveled;
		d_errorRate = d_currError - d_prevError;
		d_accumError += d_errorRate * currDt;
		d_PIDValue = d_kP * d_currError + d_kI * d_accumError;

		t_prevError = t_currError;
		t_currError = t_target - orientation;
		t_errorRate = t_currError - t_prevError;
		t_accumError += t_errorRate * currDt;
		t_PIDValue = t_kP * t_currError + t_kI * t_accumError;

		if(d_PIDValue > 100) {
			d_power = drivePower;
		}
		else if(d_PIDValue < -100) {
			d_power = -drivePower;
		}
		else if(d_PIDValue < 100 && d_PIDValue > 20) {
			d_power = (int)((float)d_PIDValue);
		}
		else if(d_PIDValue > -100 && d_PIDValue < -20) {
			d_power = (int)((float)d_PIDValue);
		}
		else if(d_PIDValue < 20) {
			d_power = 20;
		}
		else if(d_PIDValue < -20) {
			d_power = -20;
		}

		if(t_PIDValue > 30) {
			r_errorPower = 30;
			l_errorPower = 0;
		}
		else if(t_PIDValue < -30) {
			r_errorPower = 0;
			l_errorPower = 30;
		}
		else if(t_PIDValue < 30 && t_PIDValue > 0) {
			r_errorPower = t_PIDValue;
			l_errorPower = 0;
		}
		else if(t_PIDValue > -30 && t_PIDValue < 0) {
			r_errorPower = 0;
			l_errorPower = t_PIDValue;
		}

		if(abs(d_currError) < 50) {
			d_power = 0;
			r_errorPower = 0;
			l_errorPower = 0;
			isMoving = false;
			isFineTune = true;
		}
		motor[leftWheel] = d_power + l_errorPower;
		motor[rightWheel] = d_power + r_errorPower;
	}
	if(isFineTune) {
		Time_ClearTimer(fineTuneTimer);
		while(Time_GetTime(fineTuneTimer) < fineTuneTime) {
			if(d_target - d_distanceTraveled > 0)	{
				d_power = 15;
			}
			else if(d_target - d_distanceTraveled < 0) {
				d_power = -15;
			}
			else	{
				d_power = 0;
			}
			if(t_target - orientation > 0)	{
				r_errorPower = 2;
				l_errorPower = 0;
			}
			else if(t_target - orientation < 0) {
				r_errorPower = 0;
				l_errorPower = 2;
			}
			else	{
				r_errorPower = 0;
				l_errorPower = 0;
			}
			motor[leftWheel] = d_power + l_errorPower;
			motor[rightWheel] = d_power + r_errorPower;
		}
	}
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
	wait1Msec(waitTime);
}
*/
