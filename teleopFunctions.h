#include "includes.h"

/////////////////////////////// Changable Variables ////////////////////

// servos
const int startIRPos = 35;
const int startPosClampR = 110;
const int startPosClampL = 110;
const int endPosClampR = 240;
const int endPosClampL = 10;
const int startPosDrop = 70;
const int endPosDrop = 30;
// powers
const int maxPower = 100;
const int liftPower = 100;
const int dropPower = -40;
const int pickupPower = 65;
const int stopPower = 0;
bool isFullPower = true;
// goal positions
const int goalPosLow = 40;
const int goalPosMid = 70;
const int goalPosHigh = 100;
const int goalPosCenter = 130;
// PID
const int pulseValue = 280;
const float l_gearRatio = 1.0;
const float l_wheelDiam = 10.0; // in cm
const float l_circumference = l_wheelDiam * PI;

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
void raiseLift();
// sets the lift position
void setLift(float pos);

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
task t_setLiftLow();
task t_setLiftMiddle();
task t_setLiftHigh();
task t_setLiftCenter();

////////////////////////// Function Definitions /////////////////////

// gets the robot ready
void servoPrep() {
	servo[irServo] = startIRPos; // resets ir servo
	servo[dropServo] = startPosDrop;		// resets drop servo
}

// sets lift position
void setLift(float pos) {
    float target;
    bool isLifting = true;
    int power = 0.0;
    int timer = 0.0;
	float l_power;
	float kP = 0.3;
	float kI = 10.0;
	float totalDt = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float i_error = 0.0;
	float i_errorValue = 0.0;

    if(pos > l_distanceTraveled) {
        target = l_distanceTraveled + (pos / l_circumference /l_gearRatio) * pulseValue;
    } else if (pos < l_distanceTraveled) {
        target = l_distanceTraveled - (pos / l_circumference /l_gearRatio) * pulseValue;
    } else {
        target = l_distanceTraveled;
    }
    Time_ClearTimer(timer);
    while(isLifting) {
        currDt = Time_GetTime(timer) / 1000 - totalDt;
        totalDt += currDt;
        prevError = currError;
        currError = target - l_distanceTraveled;
        errorRate = prevError - currError;

        i_error = errorRate * currDt;
        i_errorValue += i_error;

        PIDValue = kP * currError + kI * i_errorValue;

		if(PIDValue > 500) {
			l_power = maxPower;
		}
		else if(PIDValue < 500) {
			l_power = -maxPower;
		}
		else if(PIDValue > 150) {
			l_power = 40;
		}
		else if(PIDValue < -150) {
			l_power = -40;
		}
		else {
			l_power = 0;
			isLifting = false;
		}
		motor[liftMotor] = l_power;
        wait1Msec(50);
	}
	motor[liftMotor] = 0;

}

// raises the lift
void raiseLift(float distance) {
	float target = l_distanceTraveled + (distance / l_circumference / l_gearRatio) * pulseValue;
    bool isLifting = true;
    int power = 0.0;
    int timer = 0.0;
	float l_power;
	float kP = 0.3;
	float kI = 10.0;
	float totalDt = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float i_error = 0.0;
	float i_errorValue = 0.0;
	Time_ClearTimer(timer);

    while(isLifting) {
        currDt = Time_GetTime(timer) / 1000 - totalDt;
        totalDt += currDt;
        prevError = currError;
        currError = target - l_distanceTraveled;
        errorRate = prevError - currError;

        i_error = errorRate * currDt;
        i_errorValue += i_error;
        PIDValue = kP * currError + kI * i_errorValue;

		if(PIDValue > 500) {
			l_power = maxPower;
		}
		else if(PIDValue < 500) {
			l_power = -maxPower;
		}
		else if(PIDValue > 150) {
			l_power = 40;
		}
		else if(PIDValue < -150) {
			l_power = -40;
		}
		else {
			l_power = 0;
			isLifting = false;
		}
		motor[liftMotor] = l_power;
	}
	motor[liftMotor] = 0;
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
/*task t_raiseLiftLow() {
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

// raises the lift to the center goal
task t_raiseLiftCenter() {
    raiseLift(goalPosCenter);
    wait1Msec(1);
}
*/

// raises the lift slightly
task t_lowerLift() {
	while(true) {
		motor[liftMotor] = dropPower;
	}
	wait1Msec(1);
}

// raises lift slightly
task t_raiseLift() {
	while(true) {
		motor[liftMotor] = liftPower;
	}
	wait1Msec(1);
}


// stops the lift
task t_stopLift() {
	motor[liftMotor] = stopPower;
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
	servo[clampServoR] = endPosClampR;
	servo[clampServoL] = endPosClampL;
	wait1Msec(1);
}

// raises the clamp
task t_raiseClamp() {
	servo[clampServoR] = startPosClampR;
	servo[clampServoL] = startPosClampL;
	wait1Msec(1);
}

// drops the balls
task t_dropBall() {
	servo[dropServo] = endPosDrop;
	wait1Msec(1);
}

// resets the drop servo
task t_resetDrop() {
	servo[dropServo] = startPosDrop;
	wait1Msec(1);
}

// sets the lift to the low goal pos
task t_setLiftLow() {
    setLift(goalPosLow);
    wait1Msec(1);
}

// sets the lift to the middle goal pos
task t_setLiftMiddle() {
    setLift(goalPosMid);
    wait1Msec(1);
}

// sets the lift to the high goal pos
task t_setLiftHigh() {
    setLift(goalPosHigh);
    wait1Msec(1);
}

// sets the lift to the center goal pos
task t_setLiftCenter() {
    setLift(goalPosCenter);
    wait1Msec(1);
}
