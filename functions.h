#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     gyroSensor,     sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irSensor,       sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     pickupMotor,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightWheel,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     Blah,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_1,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     liftMotor,     tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    irServo,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "includes.h"

void turnLeft(float degrees);
void turnRight(float degrees);
void driveForward(float distance);
void driveBackward(float distance);
void checkIR();

float wheelSize = 14.6; // in cm
float vel_curr = 0.0;
float vel_prev = 0.0;
float dt = 0.0;
float timer_gyro = 0;
float heading = 0.0;
float error;
int fullPower = 100;
bool irDetected = false;

task Gyro();
task raiseIR();
task lowerIR();
task readIR();


task Gyro() {
	Time_ClearTimer(timer_gyro);
	while (true) {
		vel_prev = vel_curr;
		dt = Time_GetTime(timer_gyro)/1000.0;
		Time_ClearTimer(timer_gyro);
		vel_curr = (float)HTGYROreadRot(gyroSensor);
		heading += (vel_prev+vel_curr)*0.5*dt;
		wait1Msec(1);
	}
}

task raiseIR() {
	servo[irServo] = 255;
	wait1Msec(1);
}

task lowerIR() {
	servo[irServo] = 0;
	wait1Msec(1);
}

task readIR() {
	while(!irDetected) {
		checkIR();
		wait1Msec(1);
	}
}

void turnLeft(float degrees) {
	bool isTurning = true;
	float startOrientation = heading;
	float currentOrientation = heading;
	float target = startOrientation + degrees;
	float power, power_neg;
	float kP = 5.7;
	bool isFineTune = false;
	float finish_timer = 0.0;
	float timer_timeout = 0.0;
	float timeout_threshold = 3000.0;

	Time_ClearTimer(timer_timeout);

	while(isTurning && Time_GetTime(timer_timeout) < timeout_threshold) {
		currentOrientation = heading;
		error = target - currentOrientation;
		if(error > 60)
			power = fullPower;
		else if(error < 60)
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
		Motor_SetPower(power_neg, leftWheel);
		Motor_SetPower(power, rightWheel);
		if(abs(error) < 2.5) {
			if(isFineTune == false) {
				Time_ClearTimer(finish_timer);
				isFineTune = true;
			}
			else if(Time_GetTime(finish_timer) > 500)
				isTurning = false;
		}
		if(Time_GetTime(timer_timeout) > timeout_threshold) {
			isTurning = false;
			while(true) {
				Motor_SetPower(0, leftWheel);
				Motor_SetPower(0, rightWheel);
				Time_Wait(1);
			}
		}
	}

	Motor_SetPower(0, leftWheel);
	Motor_SetPower(0, rightWheel);
}

void turnRight(float degrees) {
	turnLeft(-degrees);
}

void driveForward(float distance) {
	float target = distance;
	float kP = 0.03;
	float power = 0.0;
	bool isMoving = true;
	int timer_timeout = 0;
	int timer_threshold = 4000;
	float pos_avg;
	Time_ClearTimer(timer_timeout);

	Motor_ResetEncoder(leftWheel);
	Motor_ResetEncoder(rightWheel);

	while(isMoving) {
		pos_avg = (Motor_GetEncoder(leftWheel) - Motor_GetEncoder(rightWheel)) / 2.0;
		error = target - pos_avg;
		if(error > 3000)
			power = g_FullPower;
		else if(error < -3000)
			power = -g_FullPower;
		else
			power = kP*error;
		if(abs(power) < 10) {
			if(power > 0)
				power = 15;
			else if(power < 0)
				power = -15;
		}
		power = Math_Limit(power, g_FullPower);
		Motor_SetPower(power, leftWheel);
		Motor_SetPower(power, rightWheel);
		if(abs(error) < 150)
			isMoving = false;
		if(Time_GetTime(timer_timeout) > timer_threshold) {
			isMoving = false;
			Motor_SetPower(0, leftWheel);
			Motor_SetPower(0, rightWheel);
			Time_Wait(50);
		}
	}
}

void driveBackward(float distance) {
	driveForward(-distance);
}

void checkIR() {
	if(SensorValue[irSensor] == 0)
		irDetected = true;
	else
		irDetected = false;
	wait1Msec(1);
}
