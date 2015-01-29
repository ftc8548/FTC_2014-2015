#include "includes.h"

/////////////////////////////// Changable Variables ////////////////////

// servos
const int startIRPos = 35;
const int startPosClampR = 110;
const int startPosClampL = 110;
const int startPosClamp = 0;
const int endPosClampR = 240;
const int endPosClampL = 10;
const int endPosClamp = 200;
const int startPosDrop = 70;
const int endPosDrop = 30;
// powers
const int raisePower = 100;
const int lowerPower = 40;
const int dropPower = -40;
const int pickupPower = 65;
const int stopPower = 0;
bool isFullPower = true;
// goal positions
const int goalPosLow = 40;
const int goalPosMid = 70;
const int goalPosHigh = 100;
// PID
const int regPulseValue = 360;
const float l_gearRatio = 1.0;
const float l_wheelDiam = 10.0; // in cm
const float l_circumference = l_wheelDiam * PI;
const int fineTuneTime = 1500;

////////////////////////////// Don't Change These Variables //////////////

int drivePowerR = 0;
int drivePowerL = 0;
bool isLift = false;
bool isPickup = false;
bool isTurning = false;
float l_distanceTraveled = 0.0;

/////////////////////////// Function Declarations //////////////////////

// gets the robot ready
void servoPrep();
// raises lift
void raiseLift(float distance);
// lowers the lift a distance
void lowerLift(float distance);

/////////////////////////// Task Declarations ///////////////////////

task t_liftEncoder();
task t_lowerLift();
task t_raiseLift();
task t_stopLift();
task t_startPickup();
task t_reversePickup();
task t_stopPickup();
task t_dropClamp();
task t_raiseClamp();
task t_dropBall();
task t_resetDrop();
task t_raiseLiftLow();
task t_raiseLiftMiddle();
task t_raiseLiftHigh();
task t_lowerLiftLow();
task t_lowerLiftMiddle();
task t_lowerLiftHigh();

////////////////////////// Function Definitions /////////////////////

// gets the robot ready
void servoPrep() {
	servo[irServo] = startIRPos; // resets ir servo
	servo[goalServo] = startPosDrop;		// resets drop servo
}

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
	isLift = true;

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
			if(target - l_distanceTraveled > 0) {
				l_power = 20;
			}
			else if(target - l_distanceTraveled < 0) {
				l_power = -10;
			}
			else if(target - l_distanceTraveled == 0) {
				l_power = 0;
			}
			motor[liftMotor] = l_power;
		}
	}
	motor[liftMotor] = 0;
	isLift = false;
	wait1Msec(1);
}

// lowers lift a set position
void lowerLift(float distance) {
	raiseLift(-distance);
}

/////////////////////////////// Task Definitions ////////////////////

// tells position of the lift
task t_liftEncoder() {
	while(true) {
		l_distanceTraveled = (float) (Motor_GetEncoder(liftMotor));
		wait1Msec(1);
	}
}

// lowers the lift to the bottom goal
task t_raiseLiftLow() {
    raiseLift(goalPosLow);
    wait1Msec(1);
}

// raises the lift to the middle goal
task t_raiseLiftMiddle() {
    raiseLift(goalPosMid);
    wait1Msec(1);
}

// raises the lift to the high goal
task t_raiseLiftHigh() {
    raiseLift(goalPosHigh);
    wait1Msec(1);
}

// lowers lift from middle goal to ground
task t_lowerLiftLow() {
	lowerLift(goalPosLow);
}

// lowers lift from middle goal to ground
task t_lowerLiftMiddle() {
	lowerLift(goalPosMid);
}

// lowers lift from high goal to ground
task t_lowerLiftHigh() {
	lowerLift(goalPosHigh);
}

// raises the lift slightly
task t_lowerLift() {
	while(true) {
		motor[liftMotor] = dropPower;
		isLift = true;
	}
	isLift = false;
	wait1Msec(1);
}

// raises lift slightly
task t_raiseLift() {
	while(true) {
		motor[liftMotor] = raisePower;
		isLift = true;
	}
	isLift = false;
	wait1Msec(1);
}


// stops the lift
task t_stopLift() {
	motor[liftMotor] = stopPower;
	isLift = false;
	wait1Msec(1);
}

// pick up balls
task t_startPickup() {
	while(true) {
		motor[firstPickupMotor] = pickupPower;
		isPickup = true;
	}
	wait1Msec(1);
}

// reverses the pickup
task t_reversePickup() {
	while(true) {
		motor[firstPickupMotor] = -pickupPower;
		isPickup = true;
	}
	wait1Msec(1);
}

task t_stopPickup() {
	motor[firstPickupMotor] = stopPower;
	isPickup = false;
	wait1Msec(1);
}

// drops the clamp
task t_dropClamp() {
	//servo[clampServoR] = endPosClampR;
	//servo[clampServoL] = endPosClampL;
	servo[clampServo] = endPosClamp;
	wait1Msec(1);
}

// raises the clamp
task t_raiseClamp() {
	//servo[clampServoR] = startPosClampR;
	//servo[clampServoL] = startPosClampL;
	servo[clampServo] = startPosClamp;
	wait1Msec(1);
}

// drops the balls
task t_dropBall() {
	servo[goalServo] = endPosDrop;
	wait1Msec(1);
}

// resets the drop servo
task t_resetDrop() {
	servo[goalServo] = startPosDrop;
	wait1Msec(1);
}
