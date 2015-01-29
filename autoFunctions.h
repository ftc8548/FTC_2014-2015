#include "includes.h"

///////////////////////// Changable Variables //////////////////////////

// servos
const int endIRPos = 130;
const int startIRPos = 35;
const int startPosClampR = 110;
const int startPosClampL = 110;
const int startPosClamp = 0;
const int endPosClampR = 240;
const int endPosClampL = 10;
const int endPosClamp = 200;
const int startPosDrop = 70;
const int endPosDrop = 30;
const int startPosCenter = 30;
const int endPosCenter = 240;
// driving powers
const int turnPower = 40;
const int drivePower = 40;
const int rampPower = 25;
const int raisePower = 100;
const int lowerPower = 40;
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
const int fineTuneTime = 1500;
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
void driveForward(float distance);
// moves the robot backward
void driveBackward(float distance);
// turns the robot right
void turnRight(float degrees);
// turns the robot left
void turnLeft(float degrees);
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

///////////////////////////// Function Definitions ///////////////////////////

// gets the servos ready
void servoPrep() {
	raiseClamp();
	resetDropGoal();
	resetDropCenter();
	lowerIR();
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
	wait1Msec(1000);
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
	//servo[clampServoL] = startPosClampL;
	//servo[clampServoR] = startPosClampR;
	servo[clampServo] = startPosClamp;
	wait1Msec(100);
}

// drops clamp servos
void dropClamp() {
	//servo[clampServoL] = endPosClampL;
	//servo[clampServoR] = endPosClampR;
	servo[clampServo] = endPosClamp;
	wait1Msec(100);
}

// drops balls into goal
void dropBallGoal() {
	servo[goalServo] = endPosDrop;
	wait1Msec(1000);
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

// moves the robot forward
void driveBackward(float distance) {
	driveForward(-distance);
}

// turns the robot to the right
void turnRight(float degrees) {
	bool isTurning = true;
	bool isFineTune = false;
	int timer = 0.0;
	int fineTuneTimer = 0.0;
	float target = orientation + degrees;
	float t_power;
	float kP = 0.9;
	float kI = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float accumError = 0.0;

	Time_ClearTimer(timer);
	while(isTurning) {
		currDt = Time_GetTime(timer) / 1000;
		Time_ClearTimer(timer);
		prevError = currError;
		currError = target - orientation;
		errorRate = prevError - currError;
		accumError += errorRate * currDt;
		PIDValue = kP * currError + kI * accumError;

		if(PIDValue > turnPower)
			t_power = turnPower;
		else if(PIDValue < -turnPower)
			t_power = -turnPower;
		else	{
			t_power = PIDValue;
		}
		if(abs(t_power) < 30) {
			if(t_power > 0)
				t_power = 30;
		else if(t_power < 0)
				t_power = -30;
		}
		if(abs(currError) < 0.5) {
			isFineTune = true;
		}
		motor[leftWheel] = t_power;
		motor[rightWheel] = -t_power;
	}
	if(isFineTune) {
		Time_ClearTimer(fineTuneTimer);
		while(Time_GetTime(fineTuneTimer) < fineTuneTime) {
			if(target - orientation > 0.1)	{
				t_power = 20;
			}
			else if(target - orientation < -0.1) {
				t_power = -20;
			}
			else	{
				t_power = 0;
			}
			motor[leftWheel] = t_power;
			motor[rightWheel] = -t_power;
		}
	}
	//nxtDisplayTextLine(3, "pwr: %d", t_power);
	//nxtDisplayTextLine(5, "error: %d", currError);
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
	wait1Msec(waitTime);
}

// turns the robot to the left
void turnLeft(float degrees) {
	turnRight(-degrees);
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
	wait1Msec(waitTime);
}

// lowers the lift
void lowerLift(float distance) {
	raiseLift(-distance);
}

// sets lift position
void setLift(float pos) {
  float target;
  bool isLifting = true;
  int timer = 0.0;
	float l_power;
	float kP = 0.3;
	float kI = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float accumError = 0.0;

    if(pos > l_distanceTraveled) {
        target = l_distanceTraveled + (pos / l_circumference /l_gearRatio) * regPulseValue;
    } else if (pos < l_distanceTraveled) {
        target = l_distanceTraveled - (pos / l_circumference /l_gearRatio) * regPulseValue;
    } else {
        target = l_distanceTraveled;
    }
    Time_ClearTimer(timer);
    while(isLifting) {
        currDt = Time_GetTime(timer) / 1000;
        Time_ClearTimer(timer);
        prevError = currError;
        currError = target - l_distanceTraveled;
        errorRate = prevError - currError;
        accumError += errorRate * currDt;
        PIDValue = kP * currError + kI * accumError;

		if(PIDValue > 500) {
			l_power = liftPower;
		}
		else if(PIDValue < 500) {
			l_power = -liftPower;
		}
		else if(PIDValue > 150) {
			l_power = 50;
		}
		else if(PIDValue < -150) {
			l_power = -50;
		}
		else if(currError < 50)	{
			l_power = 0;
			isLifting = false;
		}
		motor[liftMotor] = l_power;
    wait1Msec(1);
	}
	motor[liftMotor] = 0;
}

////////////////////////////// Task Definitions ///////////////////////////

// starts the gyro
task gyro() {
	int timer_gyro = 0;
	g_vel_curr = 0.0;
	float g_dt = 0.0;
	HTGYROstartCal(gyroSensor);
	//Joystick_WaitForStart();
	Time_ClearTimer(timer_gyro);
	while (true) {
		g_dt = (float)Time_GetTime(timer_gyro) / 1000.0;
		Time_ClearTimer(timer_gyro);
		g_vel_curr = (float)HTGYROreadRot(gyroSensor);
		orientation += g_vel_curr * g_dt;
		nxtDisplayTextLine(0, "ang: %d", orientation);
		nxtDisplayTextLine(1, "vel: %d", g_vel_curr);
		nxtDisplayTextLine(2, "cal: %d", HTGYROreadCal(gyroSensor));
		wait1Msec(1);
	}
}

// Distance Travelled by robot
task wheelEncoder() {
	d_distanceTraveled = 0.0;
	float tickRight, tickLeft;
  while(true) {
    d_distanceTraveled = (float) (Motor_GetEncoder(leftWheel) + Motor_GetEncoder(rightWheel)) / 2.0;
    tickLeft = Motor_GetEncoder(leftWheel);
    tickRight = Motor_GetEncoder(rightWheel);
    nxtDisplayTextLine(3, "dist: %d", d_distanceTraveled);
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
		HTIRS2setDSPMode(irSensor, 1200);
		HTIRS2readAllACStrength(irSensor, irA, irB, irC, irD, irE);
		// 0 to 50, 0 being no and 50 being super close
		nxtDisplayTextLine(4, "irA: %d", irA);
		nxtDisplayTextLine(5, "irB: %d", irB);
		nxtDisplayTextLine(6, "irC: %d", irC);
		nxtDisplayTextLine(7, "irD: %d", irD);
		nxtDisplayTextLine(8, "irE: %d", irE);
		if(irC != 0) {
			irDetected = true;
		}
		wait1Msec(1);
	}
}
