#include <stdio.h>
#include <iostream>
#include <time.h>
#include <vector>
#include <wiringPi.h>
#include <pthread.h>
//#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/opencv.hpp" 
//#include <raspicam/raspicam_cv.h>

#include "MotorControl.h"

#define LEFT_IR_PIN 	    10
#define RIGHT_IR_PIN 	    11

#define LEFT_TOP_IR_PIN     27
#define RIGHT_TOP_IR_PIN    26

#define TRIG_PIN 			28
#define ECHO_PIN 			29

// privilege levels
#define SHAPE_DETECTOR_PRIVILEGE_LEVEL		2
#define TOP_IR_PRIVILEGE_LEVEL 		    	3
#define BOTTOM_IR_PRIVILEGE_LEVEL 		1

// Turn commands
#define STOP 				0
#define FORWARD 			1
#define LEFT				2 // go forward and turn left
#define LEFT_POINT			3 // stay and turn left
#define RIGHT				4
#define RIGHT_POINT			5
#define SPEED_DOWN			6
#define SPEED_UP			7
#define STOP_DELAY			8
#define LEFT_CURVE_TURN_20_60		9
#define LEFT_CURVE_TURN_20_75		11
#define BACK				12
#define NOTHING 			-1 // give way to other cammands

//using namespace cv;
using namespace std;

void initSensor();

void *bottomIR(void *);
void *topIR(void *);
void *shape(void *);

//void detect(Mat src);
void setCommand(int, int);

MotorControl motor;

/// Global Flags for motor control
static int privilege_level = 0;
static int command = NOTHING; // command to imlement

int getDistance();

int main(void) {

	if(wiringPiSetup() == -1) return 0;

	initSensor();

	pthread_t t1, t2, t3, t4;
	pthread_create(&t1, NULL, bottomIR, (void *) 0);
	delay(15);
	pthread_create(&t2, NULL, topIR, (void *) 0);
	delay(15);

	motor.setSpeed(40);
	motor.setTurnSpeed(80);
	// main thread - motor control
	while(true)	{
		if(command == STOP) {
			motor.stop();
		} else if(command == STOP_DELAY) {
			motor.stop();
			delay(6000);
			//motor.goForward();
			//delay(200);
		} else if(command == FORWARD) {
			motor.goForward();
			//motor.stop();
			//command == NOTHING;
		} else if(command == BACK) {
			motor.goBack();
		} else if(command == LEFT) {
			motor.goLeftSwing();
			//motor.stop();
			//command == NOTHING;
		} else if(command == RIGHT) {
			motor.goRightSwing();
			//motor.stop();
			//command == NOTHING;
		} else if(command == RIGHT_POINT) {
			motor.goRightPoint();
			cout << "Right Point" << endl;
			//delay(100);
			//command = NOTHING;
		} else if(command == LEFT_POINT) {
			motor.goLeftPoint();
			cout << "Left Point" << endl;
			//delay(100);
			//command = NOTHING;
		} else if(command == LEFT_CURVE_TURN_20_60) {
			motor.goCurveTurn(20, 90);
		} else if(command == SPEED_DOWN) {
			motor.setSpeed(motor.getSpeed() - 10);
		} else if(command == SPEED_UP) {
			motor.setSpeed(motor.getSpeed() + 10);
		} 

		//command = NOTHING;
		//delay(500);
	}

	pthread_exit(NULL);

	return 0;

} // end of main



void initSensor() {
	pinMode(LEFT_IR_PIN, INPUT);
	pinMode(RIGHT_IR_PIN, INPUT);

	pinMode(LEFT_TOP_IR_PIN, INPUT);
	pinMode(RIGHT_TOP_IR_PIN, INPUT);

	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
}

// Bottom Ir Sensors

void *bottomIR(void *unnec) {

	int nValue = 0, oValue = 0, nLValue = 0, nRValue = 0, count = 0;



	while(true) {

		delay(50);

		nLValue = digitalRead(LEFT_IR_PIN);
		nRValue = digitalRead(RIGHT_IR_PIN);
		nValue  = nLValue + nRValue;
	
		//cout << "bot left - " << nLValue << ", right - " << nRValue << endl;

		if (nLValue == 0 && nRValue == 1) {
		    setCommand(BOTTOM_IR_PRIVILEGE_LEVEL, RIGHT_POINT);
		} else if(nLValue == 1 && nRValue == 0) {
		    setCommand(BOTTOM_IR_PRIVILEGE_LEVEL, LEFT_POINT);
		} else if (oValue < 2 && nValue == 2) {
			count++;

			if(count > 2) {
				setCommand(BOTTOM_IR_PRIVILEGE_LEVEL, STOP);
			} else {
				//setCommand(BOTTOM_IR_PRIVILEGE_LEVEL, FORWARD);	
			}
		} else {
			setCommand(BOTTOM_IR_PRIVILEGE_LEVEL, FORWARD);	
		}

		oValue = nValue;
	}

	pthread_exit(NULL);

} // end of bottomIR



// Top Ir Sensors

void *topIR(void *unnec) {

	int nValue = 0, oValue = 0, tLValue = 0, tRValue = 0, count = 0;
	int cache = LEFT;

	while(1) {
		delay(200);

		tLValue = digitalRead(LEFT_TOP_IR_PIN);
		tRValue = digitalRead(RIGHT_TOP_IR_PIN);
		nValue  = tLValue + tRValue;

		//cout << "top left - " << tLValue << ", right - " << tRValue << endl;
		//cout << "n - " << nValue << ", o - " << oValue << endl;

		if(nValue == 0 && oValue != 0) {
			count++;
			cout << "count - " << count << endl;
		}
//
		if(nValue == 0) {

			if(count == 1) {
				cout<<"stop counter - "<< count <<endl;
				setCommand(TOP_IR_PRIVILEGE_LEVEL, STOP);
				delay(5000);
			} else if (count == 2) {
					setCommand(TOP_IR_PRIVILEGE_LEVEL, BACK);	
					delay(800);
					setCommand(TOP_IR_PRIVILEGE_LEVEL, RIGHT);
					delay(300);
					setCommand(TOP_IR_PRIVILEGE_LEVEL, FORWARD);
					delay(150);
			} else if (count == 3) {
					setCommand(TOP_IR_PRIVILEGE_LEVEL, BACK);	
					delay(800);
					setCommand(TOP_IR_PRIVILEGE_LEVEL, LEFT);
					delay(300);
					setCommand(TOP_IR_PRIVILEGE_LEVEL, FORWARD);
					delay(150);
			} else if (count == 4) {
					setCommand(TOP_IR_PRIVILEGE_LEVEL, RIGHT);
					delay(300);
					setCommand(TOP_IR_PRIVILEGE_LEVEL, FORWARD);
					delay(150);
			} else {
				cout<<"stop counter - "<< count <<endl;
				setCommand(TOP_IR_PRIVILEGE_LEVEL, STOP);
			}
		}  else {
			setCommand(TOP_IR_PRIVILEGE_LEVEL, NOTHING);
		}

		oValue = nValue;
	}
} // end of topIR

// UltraSonic Sensor
int getDistance() {
	int 	start_time = 0, end_time = 0;
	float	distance = 0.0;

	digitalWrite(TRIG_PIN, LOW);
	delay(500);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	while(digitalRead(ECHO_PIN)==0);
	start_time = micros();

	while(digitalRead(ECHO_PIN)==1);
	end_time=micros();

	distance = (end_time - start_time)/29.0/2;
	cout << "distance - " << distance << endl;
	
	return distance;
} // end of ultraSonic

// Check privilege level
void setCommand(int lvl, int com) {
	if(command == NOTHING) { // if current command is NOTHING then it does not matter which command to do next
		privilege_level = lvl;
		command = com;
	} else { // if current command is not NOTHING then it does matter which sensor can execute command
		if(lvl >= privilege_level) {
			privilege_level = lvl;
			command = com;
		}
	}
	//cout << "current privilege_level = " << privilege_level << ", command = " << command << endl;
}

/// FINAL COMMAND

/// New Khikmet and Voriskhon command + Mix of new Akbar and Bobur comands

// g++ sensors.cpp MotorControl.cpp LaneDetector.cpp ShapeDetector.cpp ColorSplitter.cpp CollectionAggregator.cpp Object.cpp -o sen -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lpthread -lwiringPi

/// Lane + BottomIR

// g++ sensors.cpp MotorControl.cpp LaneDetector.cpp -o sen -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lpthread -lwiringPi

/// Shape + BottomIR

// g++ sensors.cpp MotorControl.cpp ShapeDetector.cpp ColorSplitter.cpp CollectionAggregator.cpp Object.cpp -o sen -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lpthread -lwiringPi

/// Ultra + TopIR + obstacle avoidance

// g++ sensors.cpp MotorControl.cpp -o sen -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lpthread -lwiringPi





/// Old Khikmet and Voriskhon command

// g++ planB.cpp MotorControl.cpp -o planB -lwiringPi -pthread



/// Old Bobur command

// g++ sensors.cpp MotorControl.cpp ShapeDetector.cpp ColorSplitter.cpp CollectionAggregator.cpp Object.cpp -o sen -lwiringPi -pthread -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv`



/// New Bobur command

// g++ sensors.cpp ShapeDetector.cpp ColorSplitter.cpp CollectionAggregator.cpp Object.cpp -o sen -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv`



/// Akbar command

// g++ sensors.cpp ./LaneDet/LaneDetector.cpp -o sen -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lpthread -lwiringPi

