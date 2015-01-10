/*
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
*/

// ask what Motor_getEncoder returns

#include "includes.h"

///////////////////////// Changable Variables //////////////////////////

// servos
const int endIRPos = 130;
const int startIRPos = 35;
const int startPosClampR = 110;
const int startPosClampL = 110;
const int endPosClampR = 240;
const int endPosClampL = 10;
const int startPosDrop = 70;
const int endPosDrop = 30;
// driving powers
const int pickupPower = 50;
const int turnPower = 40;
const int drivePower = 40;
const int rampPower = 25;
const int liftPower = 100;
// PID wheel variables
const int pulseValue = 280;
const float d_wheelDiam = 6.985; // in cm
const float l_wheelDiam = 10.0; // in cm
const float d_circumference = d_wheelDiam * PI;
const float l_circumference = l_wheelDiam * PI;
const float d_gearRatio = 2.0;
const float l_gearRatio = 1.0;
// lift variables
const int goalPosGround = 0;
const int goalPosHigh = 80;
const int goalPosCenter = 120;

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
// drops clamp servos
void dropClamp();
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
// checks the ir sensor for the beacon
void checkIR();
// raises the lift to the goal
void raiseLift(float distance);
// lowers the lift
void lowerLift(float distance);
// sets the lift position
void setLift(float pos);

/////////////////////////// Task Declarations ///////////////////////////////

// starts the gyro
task a_gyro();
// tells how far robot has travelled
task a_wheelEncoder();
// tells where the lift is
task a_liftEncoder();
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
// drops balls
task a_dropBall();
// resets drop
task a_resetDrop();
// sets the lift to ground pos
task a_setLiftGround();
// sets the lift to high
task a_setLiftHigh();
// sets the lift to center
task a_setLiftCenter();

///////////////////////////// Function Definitions ///////////////////////////

// gets the servos ready
void servoPrep() {
	servo[clampServoL] = startPosClampL;
	servo[clampServoR] = startPosClampR;
	servo[dropServo] = startPosDrop;
	Task_Spawn(a_lowerIR);
}

// gets the encoders ready
void encoderPrep() {
	nMotorEncoder(leftWheel) = 0;
	nMotorEncoder(rightWheel) = 0;
	nMotorEncoder(liftMotor) = 0;
}

// drops clamp servos
void dropClamp() {
	servo[clampServoL] = endPosClampL;
	servo[clampServoR] = endPosClampR;
}

// drives the robot down ramp
void ramp(float distance) {
    float target = d_distanceTraveled - (distance / d_circumference / d_gearRatio) * pulseValue;
    bool isMoving = true;
    int power = 0.0;
    int timer = 0.0;
    float d_power;
    float kP = 3.0;
    float kI = 0.0001;
    float currDt = 0.0;
    float PIDValue = 0.0;
    float currError = 0.0;
    float prevError = 0.0;
    float errorRate = 0.0;
    float accumError = 0.0;
    Time_ClearTimer(timer);

    while(isMoving) {
    	currDt = Time_GetTime(timer) / 1000;
    	Time_ClearTimer(timer);
    	prevError = currError;
      currError = target - d_distanceTraveled;
      errorRate = currError - prevError;
    	accumError += errorRate * currDt;
    	PIDValue = kP * currError + kI * accumError;

if(PIDValue > 200) {
      	d_power = rampPower;
      }
      else if(PIDValue < -200) {
      	d_power = -rampPower;
      }
      else if(PIDValue < 200 && PIDValue > 80) {
      	d_power = (int)((float)PIDValue / 8.3);
      }
      else if(PIDValue > -200 && PIDValue < -80) {
      	d_power = -(int)((float)PIDValue / 8.3);
      }
      else if(PIDValue > 80) {
        d_power = 10;
      }
      else if(PIDValue < -80) {
        d_power = -10;
      }
      motor[leftWheel] = d_power;
      motor[rightWheel] = d_power;
      if(abs(PIDValue) < 50) {
          motor[leftWheel] = 0;
          motor[rightWheel] = 0;
          isMoving = false;
      }
      wait1Msec(1);
      nxtDisplayTextLine(5, "%d", PIDValue);
      nxtDisplayTextLine(6, "%d", currError);
    }
}



// drives the robot forward
void driveForward(float distance) {
    float target = d_distanceTraveled + (distance / d_circumference / d_gearRatio) * pulseValue;
    bool isMoving = true;
    int power = 0.0;
    int timer = 0.0;
    float d_power;
    float kP = 3.0;
    float kI = 0.0001;
    float currDt = 0.0;
    float PIDValue = 0.0;
    float currError = 0.0;
    float prevError = 0.0;
    float errorRate = 0.0;
    float accumError = 0.0;
    Time_ClearTimer(timer);

    while(isMoving) {
    	currDt = Time_GetTime(timer) / 1000;
    	Time_ClearTimer(timer);
    	prevError = currError;
      currError = target - d_distanceTraveled;
      errorRate = currError - prevError;
    	accumError += errorRate * currDt;
    	PIDValue = kP * currError + kI * accumError;

      if(PIDValue > 340) {
      	d_power = drivePower;
      }
      else if(PIDValue < -340) {
      	d_power = -drivePower;
      }
      else if(PIDValue < 340 && PIDValue > 150) {
      	d_power = (int)((float)PIDValue / 8.3);
      }
      else if(PIDValue > -340 && PIDValue < -150) {
      	d_power = (int)((float)PIDValue / 8.3);
      }
      else if(PIDValue > 150) {
        d_power = 20;
      }
      else if(PIDValue < -150) {
        d_power = -20;
      }
      motor[leftWheel] = d_power;
      motor[rightWheel] = d_power;
      if(abs(PIDValue) < 50) {
          motor[leftWheel] = 0;
          motor[rightWheel] = 0;
          isMoving = false;
      }
      wait1Msec(1);
    }
}

// moves the robot forward
void driveBackward(float distance) {
	driveForward(-distance);
}
// starts the tracker tasks
void startTrackers() {
	Task_Spawn(a_gyro);
	Task_Spawn(a_liftEncoder);
	Task_Spawn(a_wheelEncoder);
}
// turns the robot to the right
void turnRight(float degrees) {
	bool isTurning = true;
	int timer = 0.0;
	int power = 0.0;
	float target = orientation + degrees;
	float t_power;
	float kP = 0.03;
	float kI = 0.0;
	float totalDt = 0.0;
	float currDt = 0.0;
	float PIDValue = 0.0;
	float currError = 0.0;
	float prevError = 0.0;
	float errorRate = 0.0;
	float accumError = 0.0;

	Time_ClearTimer(timer);
	while(isTurning) {
		currDt = Time_GetTime(timer) / 1000 - totalDt;
		totalDt += currDt;
		prevError = currError;
		currError = target - orientation;
		errorRate = prevError - currError;
		accumError += errorRate * currDt;
		PIDValue = kP * currError + kI * accumError;

		if(abs(PIDValue) < 2.5) {
				isTurning = false;
		}
		if(PIDValue > 50)
			t_power = turnPower;
		else if(PIDValue < 50)
			t_power = -turnPower;
		else
			power = PIDValue;
		if(abs(power) < 30) {
			if(power > 0)
				t_power = 20;
			else if(power < 0)
				t_power = -30;
		}
		motor[leftWheel] = -t_power;
		motor[rightWheel] = t_power;
		wait1Msec(1);
	}
	g_vel_prev = g_vel_curr;
	motor[leftWheel] = 0;
	motor[rightWheel] = 0;
}

// turns the robot to the left
void turnLeft(float degrees) {
	turnRight(-degrees);
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
	float accumError = 0.0;

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
		else {
			l_power = 0;
			isLifting = false;
		}
		motor[liftMotor] = l_power;
    wait1Msec(1);
	}
	motor[liftMotor] = 0;
}

// checks the ir sensor for the beacon
void checkIR() {
	if(SensorValue[irSensor] == 0)
		irDetected = true;
	else
		irDetected = false;
	wait1Msec(1);
}

////////////////////////////// Task Definitions ///////////////////////////

// starts the gyro
task a_gyro() {
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
		nxtDisplayTextLine(0, "%d", orientation);
		nxtDisplayTextLine(1, "%d", g_vel_curr);
		wait1Msec(1);
	}
}

// Distance Travelled by robot
task a_wheelEncoder() {
	d_distanceTraveled = 0.0;
	float tickRight, tickLeft;
  while(true) {
    d_distanceTraveled = (float) (Motor_GetEncoder(leftWheel) + Motor_GetEncoder(rightWheel)) / 2.0;
    tickLeft = Motor_GetEncoder(leftWheel);
    tickRight = Motor_GetEncoder(rightWheel);
    /*nxtDisplayTextLine(2, "%d", d_distanceTraveled);
    nxtDisplayTextLine(3, "%d", tickLeft);
    nxtDisplayTextLine(4, "%d", tickRight);*/
    wait1Msec(1);
  }
}

// tells position of the lift
task a_liftEncoder() {
	while(true) {
		l_distanceTraveled = (float) (Motor_GetEncoder(liftMotor));
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
		wait1Msec(1);
	}
}

// drops the clamp
task a_dropClamp() {
	servo[clampServoR] = endPosClampR;
	servo[clampServoL] = endPosClampL;
	wait1Msec(1);
}

// starts the pickup
task a_startPickup() {
	while(true) {
		motor[firstPickupMotor] = pickupPower;
		//wait1Msec(3000);
	}
	wait1Msec(1);
}

// reverses the pickup
task a_reversePickup() {
	while(true) {
		motor[firstPickupMotor] = -pickupPower;
		//wait1Msec(3000);
	}
	wait1Msec(1);
}

// stops the pickup
task a_stopPickup() {
	motor[liftMotor] = 0;
	wait1Msec(1);
}

// sets the lift at ground pos
task a_setLiftGround() {
	setLift(goalPosGround);
}

// sets the lift at high goal pos
task a_setLiftHigh() {
  setLift(goalPosHigh);
}

// sets the lift at center goal pos
task a_setLiftCenter() {
	setLift(goalPosCenter);
}

// drops balls
task a_dropBall() {
	servo[dropServo] = endPosDrop;
}

// reset drop
task a_resetDrop() {
	servo[dropServo] = startPosDrop;
}
