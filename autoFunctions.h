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
const int andyPulseValue = 280;
const int regPulseValue = 360;
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
// ir variables
short irA, irB, irC, irD, irE;

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
void a_raiseIR();
// lowers ir
void a_lowerIR();
// raises clamp servos
void a_raiseClamp();
// drops clamp servos
void a_dropClamp();
// drops balls
void a_dropBall();
// resets drop
void a_resetDrop();
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
// checks to see if the ir detects the beacon
task a_readIR();

///////////////////////////// Function Definitions ///////////////////////////

// gets the servos ready
void servoPrep() {
	a_raiseClamp();
	a_resetDrop();
	a_lowerIR();
}

// gets the encoders ready
void encoderPrep() {
	nMotorEncoder(leftWheel) = 0;
	nMotorEncoder(rightWheel) = 0;
	nMotorEncoder(liftMotor) = 0;
}

// starts the tracker tasks
void startTrackers() {
	Task_Spawn(a_gyro);
	Task_Spawn(a_liftEncoder);
	Task_Spawn(a_wheelEncoder);
	wait1Msec(1000);
}

// raises the ir sensor
void a_raiseIR() {
	servo[irServo] = endIRPos;
	wait1Msec(100);
}

// lowers the ir sensor
void a_lowerIR() {
	servo[irServo] = startIRPos;
	wait1Msec(100);
}

// raises clamp servos
void a_raiseClamp() {
	servo[clampServoL] = startPosClampL;
	servo[clampServoR] = startPosClampR;
}

// drops clamp servos
void a_dropClamp() {
	servo[clampServoL] = endPosClampL;
	servo[clampServoR] = endPosClampR;
	wait1Msec(100);
}

// drops balls
void a_dropBall() {
	servo[dropServo] = endPosDrop;
	wait1Msec(100);
}

// reset drop
void a_resetDrop() {
	servo[dropServo] = startPosDrop;
	wait1Msec(100);
}

// drives the robot down ramp
void ramp(float distance) {
    float target = d_distanceTraveled - (distance / d_circumference / d_gearRatio) * andyPulseValue;
    bool isMoving = true;
    int power = 0.0;
    int timer = 0.0;
    float d_power;
    float kP = 3.0;
    float kI = 0.0;
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
      if(abs(currError) < 50) {
          motor[leftWheel] = 0;
          motor[rightWheel] = 0;
          isMoving = false;
      }
      wait1Msec(1);
      //nxtDisplayTextLine(5, "%d", PIDValue);
      //nxtDisplayTextLine(6, "%d", currError);
    }
}



// drives the robot forward
void driveForward(float distance) {
    float target = d_distanceTraveled + (distance / d_circumference / d_gearRatio) * andyPulseValue;
    bool isMoving = true;
    int power = 0;
    int timer = 0.0;
    float d_power;
    float r_errorPower
    float l_errorPower = 0.0;
    float kP = 2.0;
    float kI = 0.0001;
    float currDt = 0.0;
    float PIDValue = 0.0;
    float currError = 0.0;
    float prevError = 0.0;
    float errorRate = 0.0;
    float accumError = 0.0;
    float tempOrientation = orientation;
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
	if(tempOrientation > orientation) {
		r_errorPower--;
		l_errorPower++;
	}
	else if(tempOrientation < orientation) {
		r_errorPower++;
		l_errorPower--;
	}
	else if(tempOrientation == orientaion) {
		r_errorPower = 0;
		l_errorPower = 0;
	}
	else if(tempOrientation < orientation && 
      motor[leftWheel] = d_power + l_errorPower;
      motor[rightWheel] = d_power + r_errorPower;
      if(abs(currError) < 50) {
          motor[leftWheel] = 0;
          motor[rightWheel] = 0;
          isMoving = false;
      }
      wait1Msec(1);
    }
    wait1Msec(500);
   	if(orientation!=tempOrientation) {
   		while(orientation - tempOrientation >= 0.05) {
   			motor[rightWheel] = 25;
   		}
   		motor[rightWheel] = 0;
   		while(tempOrientation - orientation >= 0.05) {
   			motor[leftWheel] = 25;
   		}
   		motor[leftWheel] = 0;
   	}
}

// moves the robot forward
void driveBackward(float distance) {
	driveForward(-distance);
}

// turns the robot to the right
void turnRight(float degrees) {
	bool isTurning = true;
	int timer = 0;
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

		if(PIDValue > 50)
			t_power = turnPower;
		else if(PIDValue < -50)
			t_power = -turnPower;
		else	{
			t_power = PIDValue;
		}
		if(abs(t_power) < 30) {
			if(t_power > 0)
				t_power = 20;
		else if(t_power < 0)
				t_power = -20;
		}
		if(abs(currError) < 0.5) {
			isTurning = false;
			t_power = 0;
		}
		motor[leftWheel] = t_power;
		motor[rightWheel] = -t_power;
		//nxtDisplayTextLine(3, "pwr: %d", t_power);
		//nxtDisplayTextLine(5, "error: %d", currError);
		wait1Msec(1);
	}
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
task a_gyro() {
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
task a_wheelEncoder() {
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
task a_liftEncoder() {
	while(true) {
		l_distanceTraveled = (float) (Motor_GetEncoder(liftMotor));
		wait1Msec(1);
	}
}

// checks the ir for the beacon
task a_readIR() {
	while(true) {
		HTIRS2setDSPMode(1200);
		HTIRS2readAllACStrength(irSensor, irA, irB, irC, irD, irE);
		// 0 to 50, 0 being no and 50 being super close
		nxtDisplayTextLine(4, "irA: %d", irA);
		nxtDisplayTextLine(5, "irB: %d", irB);
		nxtDisplayTextLine(6, "irC: %d", irC);
		nxtDisplayTextLine(7, "irD: %d", irD);
		nxtDisplayTextLine(8, "irE: %d", irE);
		wait1Msec(1);
		
	}
}
