#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(Sensor, in2,    leftClawPot,    sensorPotentiometer)
#pragma config(Sensor, in3,    rightClawPot,   sensorPotentiometer)
#pragma config(Sensor, in4,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl5,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  rightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port1,           leftClaw,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           leftDriveFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           leftDriveBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           leftLiftSingle, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           leftLiftDouble, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           rightLiftDouble, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           rightLiftSingle, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           rightDriveBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rightDriveFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          rightClaw,     tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"


enum auton_state {AUTON_COMP, AUTON_SKILLS};

auton_state auton = AUTON_COMP;



int clamp(int value, int low, int high)
{
	if (value < low) {
		return low;
	} else if (value > high) {
		return high;
	} else {
		return value;
	}
}

int absClamp(int value, int innerLimit, int outerLimit)
{
	int absValue = abs(value);
	if (innerLimit <= absValue && absValue <= outerLimit) {
		return value;
	} else if (value >= 0) {
		return clamp(value, innerLimit, outerLimit);
	} else {
		return clamp(value, -outerLimit, -innerLimit);
	}
}

void setMotorSignal(int leftSignal, int rightSignal)
{
	motor[leftDriveFront]  = leftSignal;
	motor[leftDriveBack]   = leftSignal;

	motor[rightDriveFront] = rightSignal;
	motor[rightDriveBack]  = rightSignal;
}

float calculatePower(float error)
{
	if (error < -0.1)
		return -calculatePower(-error);

	return 0.1 * error;
}

void drivePID(int err){
	//This converts err from tenths of inch to regular ticks
	err = (35.3 * err) / 10;


	setMotorSignal(0,0);
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
	int leftError, rightError, leftMotor, rightMotor,error;
	int consis = 70;
	const int tolerance = 5;
	const float kP = 0.5,kD = 0,kI = 0;
	bool isDone = false;

	while(abs(SensorValue[leftEncoder]) < abs(err) - tolerance || abs(SensorValue[rightEncoder]) < abs(err) - tolerance){
		if(abs(SensorValue[leftEncoder]) < abs(err) - tolerance || abs(SensorValue[rightEncoder]) < abs(err) - tolerance){
			leftError = err - SensorValue[leftEncoder];
			rightError = err - SensorValue[rightEncoder];
			error = leftError - rightError;
			rightMotor += consis + error * kP;
			leftMotor -= consis + error * kP;
			setMotorSignal(leftMotor,rightMotor);
			wait1Msec(50);
		}
		if(abs(err) - abs(SensorValue[leftEncoder]) < 30){
			leftMotor -= leftMotor * 0.1;
			rightMotor -= rightMotor * 0.1;
		}

	}
	setMotorSignal(0,0);
}
void autoDrive(int left, int right, int limit)
{
	setMotorSignal(0,0);
	//Clear encoders
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;

	int leftError, rightError, leftSignal, rightSignal;

	const int tolerance = 5;
	const float leftRatio = 1.0, rightRatio = 0.95;

	do {
		wait1Msec(10);

		leftError = left - SensorValue[leftEncoder];
		rightError = right - SensorValue[rightEncoder];

		leftSignal = leftRatio * calculatePower(leftError);
		rightSignal = rightRatio * calculatePower(rightError);

		leftSignal = absClamp(leftSignal, 40, limit);
		rightSignal = absClamp(rightSignal, 40, limit);

		setMotorSignal(leftSignal, rightSignal);

	} while (abs(SensorValue[leftEncoder]) < (abs(left) - tolerance) || abs(SensorValue[rightEncoder]) < (abs(right) - tolerance));
	setMotorSignal(0,0);
}

int desiredPosition = 0;




task autoLift(){
	const int LIFT_UP   = 4095;
	const int LIFT_DOWN = 400;
	int div;
	//Feedback control variables
	int liftError = 0;
	int liftSignal = 0;
	const float LIFT_K_P = 0.3; 	//Proportional constant

	while(true){
		if(abs(desiredPosition - SensorValue[liftPot]) > (float) 20 * (float) 11.6)
		{
		  	div = 1;
		}else if(abs(desiredPosition - SensorValue[liftPot]) > (float) 5 * (float) 11.6)
		{
		  div = 5;
		}else
		{
		 	div = 10;
	  }

		if(desiredPosition == 0){ //Bring to bottom
			if(SensorValue[liftPot] > LIFT_DOWN + 100){
				liftSignal = -127;
			}else if(SensorValue[liftPot] > LIFT_DOWN){
				liftSignal = -60;
			}else{
				liftSignal = -8;
			}
		}else if(desiredPosition > SensorValue[liftPot]){ //Bring down
			liftSignal = ((desiredPosition - SensorValue[liftPot])) / div;
		}else{ //Coast down
			liftSignal = 0;
		}

		//Power motors
		motor[leftLiftSingle]  = liftSignal;
		motor[leftLiftDouble]  = liftSignal;
		motor[rightLiftSingle] = liftSignal;
		motor[rightLiftDouble] = liftSignal;

		//Wait for good measure ??
	  wait1Msec(30);
	}
}

//Use in case potentiometer values need to be modified
//Usage: potReverse(SensorValue[potentiometer])
int potReverse(int value){
	return 1000 - value;
}

enum claw_state_t {CLAW_MANUAL, CLAW_CLOSE, CLAW_OPEN};

claw_state_t clawState = CLAW_MANUAL;

task clawControl(){
	//Opened and closed positions
	const int LEFT_CLOSED  = 400; //Pot value @ left  claw closed position 373
	const int LEFT_OPENED  = 1750;  //Pot value @ left  claw opened position
	const int RIGHT_CLOSED = 300; //Pot value @ right claw closed position 240s
	const int RIGHT_OPENED = 1950;  //Pot value @ right claw opened position

	//Joystick command
	int leftSignal = 0;
	int rightSignal = 0;

	while(true){

		if(vexRT[Btn6U] == 1 || clawState == CLAW_CLOSE){ //Command close
			if(SensorValue[leftClawPot] > LEFT_CLOSED){
				leftSignal = -127;
			}else{
				leftSignal = 0;
			}
			if(SensorValue[rightClawPot] > RIGHT_CLOSED){
				rightSignal = -127;
			}else{
				rightSignal = 0;
			}
		}else if(vexRT[Btn6D] == 1 || clawState == CLAW_OPEN){ //Command open
			if(SensorValue[leftClawPot] < LEFT_OPENED){
				leftSignal = 127 - SensorValue[leftClawPot] / 16;
			}else{
				leftSignal = 0;
			}
			if(SensorValue[rightClawPot] < RIGHT_OPENED){
				rightSignal = 127 - SensorValue[rightClawPot] / 16;
			}else{
				rightSignal = 0;
			}
		}else if(vexRT[Btn8R] == 1){ //Command open
			leftSignal  = 127;
			rightSignal = 127;
		}else{
			leftSignal  = 0;
			rightSignal = 0;
		}

		//Motor control with P.I. controls
		motor[leftClaw]  = leftSignal;
		motor[rightClaw] = rightSignal;

		//Wait for good measure :)
	  wait1Msec(25);
	}
}

enum lift_state_t {LIFT_MANUAL, LIFT_RAISE, LIFT_LOWER};
lift_state_t liftState = LIFT_MANUAL;

int desiredLiftPosition;

void setLiftPosition(int potValue)
{
	liftState = LIFT_MANUAL;
	desiredLiftPosition = potValue;
}

		 // integral = integral + ((desiredLiftPosition - SensorValue[liftPot])*0.03);
			//liftSignal = (LIFT_K_P*((desiredLiftPosition - SensorValue[liftPot]))) + (integral * LIFT_K_I);

task liftControl(){
	//Lowered and raised positions
	const int LIFT_UP   = 4095; //Pot value @ raised position
	const int LIFT_DOWN = 400; //Pot value @ lowest position

	const int PID_UP = 0;
	const int PID_DOWN = 0;

	int E_STOP = 2215;
	//Joystick command
	desiredLiftPosition = SensorValue[liftPot]; //Initialized with pot value @ lowest position
	int liftSignal = 0;
	int integral = 0;
	//Feedback control variables
	int liftError = 0;
	int liftErrorAccumulator = 0;
	const float LIFT_K_P = 0.3; 	//Proportional constant
	const float LIFT_K_I = 0; 	//Integral constant
	int div = 0;
	while(true){

		if(vexRT[Btn7U]){
			E_STOP = 2215;
		}
		if(vexRT[Btn7D]){
			E_STOP = 10000;
		}


		if((vexRT[Btn5U] == 1 || liftState == LIFT_RAISE) && SensorValue[liftPot] < LIFT_UP && SensorValue[liftPot] < E_STOP){ //Command raise lift
			liftSignal = 127;
			desiredLiftPosition = SensorValue[liftPot];
		}else if((vexRT[Btn5D] == 1 || liftState == LIFT_LOWER) && SensorValue[liftPot] > LIFT_DOWN + 100){ //Command lower lift
			liftSignal = -127;
			desiredLiftPosition = SensorValue[liftPot];
		}else if((vexRT[Btn5D] == 1 || liftState == LIFT_LOWER) && SensorValue[liftPot] > LIFT_DOWN ){ //Command lower lift
			liftSignal = -60;
		}else if(SensorValue[liftPot] < LIFT_DOWN){ //Command lower lift
			liftSignal = -8;
		}else{
		  integral = integral + ((desiredLiftPosition - SensorValue[liftPot])*0.03);
		  if(abs(desiredLiftPosition - SensorValue[liftPot]) > (float) 30 * (float) 11.6 && SensorValue[liftPot] < desiredLiftPosition)
		  {
		  	div = 1;
		  }else if(abs(desiredLiftPosition - SensorValue[liftPot]) > (float) 10 * (float) 11.6  && SensorValue[liftPot] < desiredLiftPosition)
		  {
		  	div = 3;
		  }else //if(SensorValue[liftPot] < desiredLiftPosition)
		  {
		  	div = 7;
		  }

			liftSignal = ((desiredLiftPosition - SensorValue[liftPot])) / div;

			if(SensorValue[liftPot] < PID_DOWN){
			//	liftSignal = 15;
			}
			else if(SensorValue[liftPot] > PID_UP){
				//liftSignal = -15;
			}


		}

		//Power motors
		motor[leftLiftSingle]  = liftSignal;
		motor[leftLiftDouble]  = liftSignal;
		motor[rightLiftSingle] = liftSignal;
		motor[rightLiftDouble] = liftSignal;

		//Wait for good measure :)
	  wait1Msec(30);
	}
}

void lift(int goal,int speed, bool stay){
	clearTimer(T1);
	bool dir;
	bool there = false;
	while(there == false)
	{
	if(SensorValue[liftPot] < goal){
		dir = true;
		motor[leftLiftSingle]  = speed;
		motor[leftLiftDouble]  = speed;
		motor[rightLiftSingle] = speed;
		motor[rightLiftDouble] = speed;
	}
	else{
		dir = false
		motor[leftLiftSingle]  = -speed;
		motor[leftLiftDouble]  = -speed;
		motor[rightLiftSingle] = -speed;
		motor[rightLiftDouble] = -speed;
	}
	if(abs(SensorValue[liftPot] - goal) < 80 && stay)
	{
		motor[leftLiftSingle]  = 20;
		motor[leftLiftDouble]  = 20;
		motor[rightLiftSingle] = 20;
		motor[rightLiftDouble] = 20;
		there = true;
	}else if(abs(SensorValue[liftPot] - goal) < 80 && stay == false)
		motor[leftLiftSingle]  = 0;
		motor[leftLiftDouble]  = 0;
		motor[rightLiftSingle] = 0;
		motor[rightLiftDouble] = 0;
		there = true;
	}
	if(time1[T1] > abs(SensorValue[liftPot]) && dir == true)
	{
	if(stay)
	{
		motor[leftLiftSingle]  = 20;
		motor[leftLiftDouble]  = 20;
		motor[rightLiftSingle] = 20;
		motor[rightLiftDouble] = 20;
	}else
	{
		motor[leftLiftSingle]  = 0;
		motor[leftLiftDouble]  = 0;
		motor[rightLiftSingle] = 0;
		motor[rightLiftDouble] = 0;
	}
		there = true;
	}
	else if(time1[T1] > abs(SensorValue[liftPot] - goal) && dir == false){

	if(stay)
	{
		motor[leftLiftSingle]  = 20;
		motor[leftLiftDouble]  = 20;
		motor[rightLiftSingle] = 20;
		motor[rightLiftDouble] = 20;
	}else
	{
		motor[leftLiftSingle]  = 0;
		motor[leftLiftDouble]  = 0;
		motor[rightLiftSingle] = 0;
		motor[rightLiftDouble] = 0;
	}
		there = true;
	}
}


void score(int err, bool score, int liftPoint, int openClaw, int maxheight){


	//Input err is tenths of inch this converts to regular ticks
	err = (35.3 * err) / 10;


	setMotorSignal(0,0);
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
	int leftError, rightError, leftMotor, rightMotor,error;
	int consis = 70;
	const int tolerance = 5;
	const float kP = 0.5,kD = 0,kI = 0;
	bool isDone = false;
	if(score){
	while(!isDone && leftError > 5){
		if(abs(SensorValue[leftEncoder]) < abs(err) - tolerance || abs(SensorValue[rightEncoder]) < abs(err) - tolerance){
			leftError = err - SensorValue[leftEncoder];
			rightError = err - SensorValue[rightEncoder];
			error = leftError - rightError;
			rightMotor += consis + error * kP;
			leftMotor -= consis + error * kP;
			setMotorSignal(leftMotor,rightMotor);
			wait1Msec(50);
		}
		else{
			setMotorSignal(0,0);
		}
		if(abs(err) - abs(SensorValue[leftEncoder]) < 30){
			leftMotor -= leftMotor * 0.1;
			rightMotor -= rightMotor * 0.1;
		}

		if(SensorValue[liftPot] < openClaw){
			if(SensorValue[leftClawPot] < 1750){
				motor[leftClaw] = 127;
			}
			else{
				motor[leftClaw] = 0;
			}
			if(SensorValue[rightClawPot] < 1950){
				motor[rightClaw] = 127;
			}
			else{
				motor[rightClaw] = 0;
			}
		}

		if(SensorValue[leftEncoder] > liftPoint && !isDone){
			if(SensorValue[liftPot] < maxheight){
				motor[leftLiftSingle]  = 127;
				motor[leftLiftDouble]  = 127;
				motor[rightLiftSingle] = 127;
				motor[rightLiftDouble] = 127;
			}
			else{
				motor[leftLiftSingle]  = -127;
				motor[leftLiftDouble]  = -127;
				motor[rightLiftSingle] = -127;
				motor[rightLiftDouble] = -127;
				wait1Msec(200);
				motor[leftLiftSingle]  = 0;
				motor[leftLiftDouble]  = 0;
				motor[rightLiftSingle] = 0;
				motor[rightLiftDouble] = 0;
				isDone = true;
			}
		}

}
}
else{
	while(SensorValue[leftEncoder] < err){

		if(abs(SensorValue[leftEncoder]) < abs(err) - tolerance || abs(SensorValue[rightEncoder]) < abs(err) - tolerance){
			leftError = err - SensorValue[leftEncoder];
			rightError = err - SensorValue[rightEncoder];
			error = leftError - rightError;
			rightMotor = consis + error * kP;
			leftError = consis + error * kP;
			setMotorSignal(leftMotor,rightMotor);
			wait1Msec(50);
		}
		if(abs(err) - abs(SensorValue[leftEncoder]) < 30){
			leftMotor -= leftMotor * 0.1;
			rightMotor -= rightMotor * 0.1;
		}
}
}
		setMotorSignal(0,0);
}











void pre_auton()
{
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

task compAuton(){
	//drivePID(tenths of an inch);
	drivePID(280);
}
task skillsAuton(){

}
task autonomous()
{

  bLCDBacklight = true;

  startTask(clawControl);
	startTask(liftControl);


	if(auton == AUTON_COMP){
		startTask(compAuton);
	}
	else if(auton == AUTON_SKILLS){
		startTask(skillsAuton);
	}


/*	setMotorSignal(-100, -100);

	autoDrive(-200, -200, 100);
	autoDrive(-400, -400, 100);

	clawState = CLAW_CLOSE;
	wait1Msec(1000);

	clawState = CLAW_MANUAL;
	*/
  //clawState = CLAW_OPEN;
  //autoDrive(2000, 2000, 70);
  //clawState = CLAW_CLOSE;
  //wait1Msec(300);

  //setLiftPosition(400);
  //wait1Msec(100);

  //autoDrive(-2000, -2000, 70);
  //wait1Msec(100);

  //autoDrive(0, -50, 45);
  //wait1Msec(100);

  //// Left side goes 400 steps backward, right side goes 400 forward, limit the speed between -70 to +70
  //autoDrive(-400, 400, 70);
  //wait1Msec(100);

  //autoDrive(-500, -500, 70);

  //setLiftPosition(800);
  //wait1Msec(200);
  //clawState = CLAW_OPEN;

	}




task usercontrol()
{

	string mainBattery;


	//Drive slew buffer
	const int SIZE = 10; //If updating SIZE, add or remove 0s from arrays below
	int oldL[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int oldR[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int sumL = 0, sumR = 0;

  //Higher control drive mapping
  int driveMap[128] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											22,23,24,25,26,27,28,28,29,29,
											30,30,31,31,32,32,33,33,34,34,
											35,35,36,36,37,37,38,38,39,39,
											40,40,41,41,42,42,43,43,44,44,
											45,45,46,46,47,47,48,48,49,49,
											50,50,51,51,52,52,53,53,54,54,
											55,55,56,56,57,57,58,58,59,59,
											60,60,61,62,63,64,65,66,67,68,
											69,70,71,72,73,74,75,76,77,78,
											79,80,81,82,83,84,85,86,87,88,
											89,90,91,92,94,96,127,127};

	//Initialize variables
	clawState = CLAW_MANUAL;
	liftState = LIFT_MANUAL;

	//Start claw and lift tasks
	startTask(clawControl);
	startTask(liftControl);

  while (true)
  {
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0, 0, "Tarj is gucci");
		displayLCDString(1,0,"Battery: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);


		//Update with current drive commands
		oldL[9] = driveMap[abs(vexRT[Ch3])] * sgn(vexRT[Ch3]);
		oldR[9] = driveMap[abs(vexRT[Ch2])] * sgn(vexRT[Ch2]);

		//Reset sums
		sumL = 0;
		sumR = 0;

		//Sum up the previous 10 drive commands
		for(int i = 0; i < SIZE - 1; i++){
			sumL += oldL[i];
			sumR += oldR[i];

			oldL[i] = oldL[i + 1];
			oldR[i] = oldR[i + 1];
		}

		sumL += oldL[9];
		sumR += oldR[9];

		//Drive motors receive moving average
		motor[leftDriveFront]  = sumL / SIZE;
	  motor[leftDriveBack]   = sumL / SIZE;
	  motor[rightDriveFront] = sumR / SIZE;
	  motor[rightDriveBack]  = sumR / SIZE;

	  //Wait for good measure :)
	  wait1Msec(25);
  }
}
