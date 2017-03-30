#pragma config(Sensor, in2,    clawL,          sensorPotentiometer)
#pragma config(Sensor, in3,    clawR,          sensorPotentiometer)
#pragma config(Sensor, in4,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in6,    status,         sensorAnalog)
#pragma config(Sensor, dgtl5,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  rightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port1,           leftClaw,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           leftDriveFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           leftDriveBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           leftLiftSingle, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           leftLiftDouble, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rightLiftDouble, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           rightLiftSingle, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           rightDriveBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rightDriveFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          rightClaw,     tmotorVex393_HBridge, openLoop, reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
enum auton_states {AUTON_COMP, AUTON_SKILLS};
auton_states auton = AUTON_COMP;
const int autonLength = 2;
int currAuton = 1;
bool inMacro;
int button = 1;
void setMotorSignal(int leftSignal, int rightSignal)
{
	motor[leftDriveFront]  = leftSignal;
	motor[leftDriveBack]   = leftSignal;
	motor[rightDriveFront] = rightSignal;
	motor[rightDriveBack]  = rightSignal;
}
void drivePID(int err){
	err = ((35.3 * err) / 10) * (15/10);
	setMotorSignal(0,0);
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
	int leftError, rightError, leftMotor, rightMotor,error;
	int consis = 70;
	const float kP = 0.5;
	while(abs(SensorValue[leftEncoder]) < abs(err) || abs(SensorValue[rightEncoder]) < abs(err)){
		if(abs(SensorValue[leftEncoder]) < abs(err) || abs(SensorValue[rightEncoder]) < abs(err)){
			leftError = err - SensorValue[leftEncoder];
			rightError = err - SensorValue[rightEncoder];
			error = leftError - rightError;
			rightMotor = consis + error * kP;
			leftMotor = consis - error * kP;
			wait1Msec(50);
		}
		if(abs(err) - abs((SensorValue[leftEncoder])) < 30){
			leftMotor -= leftMotor * 0.1;
			rightMotor -= rightMotor * 0.1;
		}
		setMotorSignal(leftMotor,rightMotor);
	}
	setMotorSignal(0,0);
}
enum claw_state_t {CLAW_MANUAL, CLAW_CLOSE, CLAW_OPEN};
claw_state_t clawState = CLAW_MANUAL;
task clawControl(){
	int CLOSED  = 420;
	int OPENED  = 1570;
	int signalL = 0;
	int signalR = 0;
	while(true){
		if(!inMacro){
			if(vexRT[Btn5D] == 1 || clawState == CLAW_CLOSE){
				if(SensorValue[clawL] > CLOSED){
					signalL = 127;
				}
				else{
					signalL = 0;
				}
				if(SensorValue[clawR] > CLOSED){
					signalR = 127;
				}
				else{
					signalR = 0;
				}
			}
			else if(vexRT[Btn5U] == 1 || clawState == CLAW_OPEN){
				if(SensorValue[clawL] < OPENED){
					signalL = -127;
				}
				else{
					signalL = 0;
				}
				if(SensorValue[clawR] < OPENED){
					signalR = -127;
				}
				else{
					signalR = 0;
				}
			}
			else{
				signalL = 0;
				signalR = 0;
			}
			motor[leftClaw]  = signalL;
			motor[rightClaw] = signalR;
			wait1Msec(25);
		}
	}
}
enum lift_state_t {LIFT_MANUAL, LIFT_RAISE, LIFT_LOWER};
lift_state_t liftState = LIFT_MANUAL;
int desiredLiftPosition;
task liftControl(){
	bool clicked6U = false;
	const int LIFT_UP   = 3930;
	const int LIFT_DOWN = 235;
	int E_STOP = 1000;
	desiredLiftPosition = SensorValue[liftPot];
	int liftSignal = 0;
	int div = 0;
	while(true){
		if(!inMacro){
			if(vexRT[Btn6U] == 1 && SensorValue[liftPot] < LIFT_UP && SensorValue[liftPot] < E_STOP){
				liftSignal = 127;
				desiredLiftPosition = SensorValue[liftPot];
				if(SensorValue[liftPot] < 600){
					clearTimer(T2);
					clicked6U = true;
				}
			}
			else if((vexRT[Btn6D] == 1 || liftState == LIFT_LOWER) && SensorValue[liftPot] > LIFT_DOWN + 100){
				liftSignal = -127;
				desiredLiftPosition = SensorValue[liftPot];
			}
			else if((vexRT[Btn6D] == 1 || liftState == LIFT_LOWER) && SensorValue[liftPot] > LIFT_DOWN ){
				liftSignal = -60;
			}
			else if(SensorValue[liftPot] < LIFT_DOWN){
				liftSignal = -8;
			}
			else{
				if(SensorValue[liftPot] > E_STOP){
					desiredLiftPosition = E_STOP - 100;
				}
				if(time1[T2] < 250 && clicked6U == true){
					desiredLiftPosition = 600;
					clicked6U = false;
				}
				if(abs(desiredLiftPosition - SensorValue[liftPot]) > (float) 30 * (float) 11.6 && SensorValue[liftPot] < desiredLiftPosition)
				{
					div = 2;
				}
				else if(abs(desiredLiftPosition - SensorValue[liftPot]) > (float) 10 * (float) 11.6  && SensorValue[liftPot] < desiredLiftPosition)
				{
					div = 3;
				}
				else
				{
					div = 6;
				}
				liftSignal = (desiredLiftPosition - SensorValue[liftPot]) / div;
			}
			if(Btn7L){
				E_STOP = 1000;
			}
			if(Btn7R){
				E_STOP = SensorFullCount[liftPot] - 11;
			}
			if(SensorValue[liftPot] > E_STOP){
		//		liftSignal -= 8;
			}
			motor[leftLiftSingle]  = liftSignal;
			motor[leftLiftDouble]  = liftSignal;
			motor[rightLiftSingle] = liftSignal;
			motor[rightLiftDouble] = liftSignal;
			wait1Msec(30);
		}
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
			dir = false;
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
		}
		else if(abs(SensorValue[liftPot] - goal) < 80 && stay == false){
			motor[leftLiftSingle]  = 0;
			motor[leftLiftDouble]  = 0;
			motor[rightLiftSingle] = 0;
			motor[rightLiftDouble] = 0;
			there = true;
		}
		if(time1[T1] > abs(SensorValue[liftPot]) && dir == true){
			if(stay){
				motor[leftLiftSingle]  = 20;
				motor[leftLiftDouble]  = 20;
				motor[rightLiftSingle] = 20;
				motor[rightLiftDouble] = 20;
			}
			else{
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
			}
			else
			{
				motor[leftLiftSingle]  = 0;
				motor[leftLiftDouble]  = 0;
				motor[rightLiftSingle] = 0;
				motor[rightLiftDouble] = 0;
			}
			there = true;
		}
	}
}
void score(int err, bool score, int liftPoint, int openClaw, int maxheight){
	err = (35.3 * err) / 10 * (15/10);
	setMotorSignal(0,0);
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
	int leftError, rightError, leftMotor, rightMotor,error;
	int consis = 70;
	const float kP = 0.5;
	bool isDone = false;
	if(score){
		while(!isDone && leftError > 5){
			if(abs(SensorValue[leftEncoder]) < abs(err)|| abs(SensorValue[rightEncoder]) < abs(err)){
				leftError = err - SensorValue[leftEncoder];
				rightError = err - SensorValue[rightEncoder];
				error = leftError - rightError;
				rightMotor = consis + error * kP;
				leftMotor = consis - error * kP;
				wait1Msec(50);
			}
			else{
				leftMotor = 0;
				rightMotor = 0;
			}
			if(abs(err) - abs(SensorValue[leftEncoder]) < 30){
				leftMotor -= leftMotor * 0.1;
				rightMotor -= rightMotor * 0.1;
			}
			if(SensorValue[liftPot] > openClaw){
				if(SensorValue[clawL] < 1600){
					motor[leftClaw] = -127;
				}
				else{
					motor[leftClaw] = 0;
				}
				if(SensorValue[clawR] > openClaw){
					motor[rightClaw] = -127;
				}
				else{
					motor[rightClaw] = 0;
				}
			}
			if((SensorValue[leftEncoder] > liftPoint || SensorValue[rightEncoder] > liftPoint) && !isDone){
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
					wait1Msec(300);
					motor[leftLiftSingle]  = 0;
					motor[leftLiftDouble]  = 0;
					motor[rightLiftSingle] = 0;
					motor[rightLiftDouble] = 0;
					isDone = true;
				}
			}
			setMotorSignal(leftMotor,rightMotor);
		}
	}
	else{
		while(SensorValue[leftEncoder] < err){
			if(abs(SensorValue[leftEncoder]) < abs(err)|| abs(SensorValue[rightEncoder]) < abs(err)){
				leftError = err - SensorValue[leftEncoder];
				rightError = err - SensorValue[rightEncoder];
				error = leftError - rightError;
				rightMotor = consis + error * kP;
				leftMotor = consis - error * kP;
				wait1Msec(50);
			}
			if(abs(err) - abs(SensorValue[leftEncoder]) < 30){
				leftMotor -= leftMotor * 0.1;
				rightMotor -= rightMotor * 0.1;
			}
			setMotorSignal(leftMotor,rightMotor);
		}
	}
	setMotorSignal(0,0);
}
void turn(int distL,int distR){
	setMotorSignal(0,0);
	int left;
	int right;
	int kp = 2;
	SensorValue[leftEncoder] = 0;
	SensorValue[rightEncoder] = 0;
	while(abs(SensorValue[leftEncoder] - distL) > 20 || abs(SensorValue[rightEncoder] - distR) > 20){
		if(abs(SensorValue[leftEncoder] - distL) > 20){
			left = (motor[leftEncoder] - distL) * kp;
		}
		if(abs(SensorValue[rightEncoder] - distR) > 20){
			right = (motor[rightEncoder] - distR) * kp;
		}
		setMotorSignal(left,right);
	}
	setMotorSignal(0,0);
}
void claw(int dist){
	if(dist > SensorValue[clawL] || dist > SensorValue[clawR]){
		while(dist > SensorValue[clawL] || dist > SensorValue[clawR]){
			if(SensorValue[clawL] < dist){
				motor[leftClaw] = -127;
			}
			else{
				motor[leftClaw] = 0;
			}
			if(SensorValue[clawR] < dist){
				motor[rightClaw] = -127;
			}
			else{
				motor[rightClaw] = 0;
			}
		}
	}
	else{
		while(dist < SensorValue[clawL]){
			if(SensorValue[clawL] > dist){
				motor[leftClaw] = 127;
			}
			else{
				motor[leftClaw] = 0;
			}
			if(SensorValue[clawR] > dist){
				motor[rightClaw] = 127;
			}
			else{
				motor[rightClaw] = 0;
			}
		}
		motor[leftClaw] = 0;
		motor[rightClaw] = 0;
	}
}
void pre_auton()
{
	bLCDBacklight = true;
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
}
task compAuton(){
	//void score(int err, bool score, int liftPoint, int openClaw, int maxheight)
	//void turn(left,right) *Note* not to tenths of inch input a raw encoder value
	//void claw(left,right)
	//void lift(height,speed,usePIDBool)
	//drivePID(tenths of an inch);
	lift(800,60,false);
	claw(1000);
	lift(150,60,false);
	claw(400);
	drivePID(280);
	turn(-1000,1000);
	claw(1600);
	drivePID(350);
	claw(750);
	lift(700,60,true);
	drivePID(-350);
	turn(-1000,1000);
	score(-150,true,-100,1000,1400);
}
task skillsAuton(){
	drivePID(-200);
	claw(1500);
	drivePID(150);
	claw(500);
	score(-300,true,-200,1000,1400);
}
task autonomous()
{
	if(currAuton == 1){
		auton = AUTON_COMP;
	}
	if(currAuton == 2){
		auton = AUTON_SKILLS;
	}
	bLCDBacklight = true;
	if(auton == AUTON_COMP){
		startTask(compAuton);
	}
	else if(auton == AUTON_SKILLS){
		startTask(skillsAuton);
	}
}
task usercontrol()
{
	bLCDBacklight = true;
	bool buttClicked;
	string powerExpander;
	string mainBattery;
	int clawLError[10];
	int clawRError[10];
	const int SIZE = 10;
	int oldL[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	int oldR[SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	int sumL = 0, sumR = 0;
	int driveMap[128] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
	clawState = CLAW_MANUAL;
	liftState = LIFT_MANUAL;
	startTask(clawControl);
	startTask(liftControl);
	while (true)
	{
		if(nLCDButtons != 0){
			button = nLCDButtons;
			if(button == 4 && !buttClicked){
				currAuton++;
				buttClicked = true;
				if(currAuton > autonLength){
					currAuton = 1;
				}
			}
		}
		if(nLCDButtons == 0){
			buttClicked = false;
		}
		if(currAuton == 1){
			auton = AUTON_COMP;
		}
		if(currAuton == 2){
			auton = AUTON_SKILLS;
		}
		clearLCDLine(0);
		clearLCDLine(1);
		if(button == 1){
			displayLCDString(0,0,"Team: 4659B");
			if(bIfiAutonomousMode){
				displayLCDString(1,0,"Autonomous");
			}
			else if(!bIfiAutonomousMode){
				displayLCDString(1,0,"User Control");
			}
		}
		if(button == 2){
			displayLCDString(0,0,"Battery: ");
			sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V');
			displayNextLCDString(mainBattery);
			displayLCDString(1,0,"Expander: ");
			sprintf(powerExpander, "%1.2f%c",(float)SensorValue[status] / 282,'V');
			displayNextLCDString(powerExpander);
			displayNextLCDString("V");
		}
		if(button == 4){
			displayLCDString(0,0,"Auton: ");
			if(auton == AUTON_COMP){
				displayLCDString(1,0,"Competition 1");
			}
			else if(auton == AUTON_SKILLS){
				displayLCDString(1,0,"Skills 1");
			}
		}




		if(vexRT[Btn7U]){
			int clawLeft;
			int clawRight;
			int lift;
			inMacro = true;
			for(int i = 0; i < 10 - 1; i++){
				clawLError[i] = clawLError[i + 1];
			}
			for(int i = 0; i < 10 - 1; i++){
				clawRError[i] = clawRError[i + 1];
			}
			clawRError[9] = SensorValue[clawR];
			clawLError[9] = SensorValue[clawL];
			if(abs(clawLError[9] - clawLError[0]) > 10){
				clawLeft = 50;
			}
			else{
				clawLeft = 0;
			}
			if(abs(clawRError[9] - clawRError[0]) > 10){
				clawRight = 50;
			}
			else{
				clawRight = 0;
			}
			if(abs(clawRError[9] - clawRError[0]) > 10){
				if(SensorValue[liftPot] < 1200){
					lift = 70;
				}
				if(SensorValue[liftPot] > 800){
					if(SensorValue[clawL] < 2000){
						clawLeft = -50;
					}
					else{
						clawLeft = 0;
					}
					if(SensorValue[clawR] < 2000){
						clawRight = -50;
					}
					else{
						clawRight = 0;
					}
				}
				if(SensorValue[liftPot] > 1000){
					lift = -40;
				}

			}
			motor[leftLiftDouble] = lift;
			motor[leftLiftSingle] = lift;
			motor[rightLiftDouble] = lift;
			motor[rightLiftSingle] = lift;
			motor[leftClaw] = clawLeft;
			motor[rightClaw] = clawRight;
		}
		else{
			inMacro = false;
		}
		oldL[9] = driveMap[abs(vexRT[Ch3])] * sgn(vexRT[Ch3]);
		oldR[9] = driveMap[abs(vexRT[Ch2])] * sgn(vexRT[Ch2]);
		sumL = 0;
		sumR = 0;
		for(int i = 0; i < SIZE - 1; i++){
			sumL += oldL[i];
			sumR += oldR[i];
			oldL[i] = oldL[i + 1];
			oldR[i] = oldR[i + 1];
		}
		sumL += oldL[9];
		sumR += oldR[9];
		motor[leftDriveFront]  = sumL / SIZE;
		motor[leftDriveBack]   = sumL / SIZE;
		motor[rightDriveFront] = sumR / SIZE;
		motor[rightDriveBack]  = sumR / SIZE;
		wait1Msec(25);
	}
}
