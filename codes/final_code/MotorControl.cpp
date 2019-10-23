#include "MotorControl.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

#define PWM_INIT_VAL       	0
#define PWM_RANGE          	100

#define MAX_SPEED       	100
#define MIN_SPEED       	0

using namespace std;

/**
 *@brief Default constructot
 *@param none
 *@return none
 */
MotorControl::MotorControl()
{
	IN1_PIN = 1; // left
	IN2_PIN = 4; // left
	IN3_PIN = 5; // right
	IN4_PIN = 6; // right
	NORM_SPEED = 45;
	TURN_SPEED = 60;
	init();
}

MotorControl::~MotorControl()
{
}

/**
 *@brief Function that initializes wiringPi, softpwm pins
 *@param none
 *@return none
 */
void MotorControl::init()
{   
	if(wiringPiSetup() == -1)

	pinMode(IN1_PIN, SOFT_PWM_OUTPUT);
	pinMode(IN2_PIN, SOFT_PWM_OUTPUT);
	pinMode(IN3_PIN, SOFT_PWM_OUTPUT);
	pinMode(IN4_PIN, SOFT_PWM_OUTPUT);

	softPwmCreate(IN1_PIN, PWM_INIT_VAL, PWM_RANGE);
	softPwmCreate(IN2_PIN, PWM_INIT_VAL, PWM_RANGE);
	softPwmCreate(IN3_PIN, PWM_INIT_VAL, PWM_RANGE);
	softPwmCreate(IN4_PIN, PWM_INIT_VAL, PWM_RANGE);
}

/**
 *@brief Function that goes forward with NORM_SPEED value fed to motors
 *@param none
 *@return none
 */
void MotorControl::goForward(void)
{
	softPwmWrite(IN1_PIN, NORM_SPEED);
	softPwmWrite(IN2_PIN, MIN_SPEED);
	softPwmWrite(IN3_PIN, NORM_SPEED);
	softPwmWrite(IN4_PIN, MIN_SPEED);
	//printf("Go Forward\n");
}

/**
 *@brief Function that goes backward with NORM_SPEED value fed to motors
 *@param none
 *@return none
 */
void MotorControl::goBack(void)
{
	softPwmWrite(IN1_PIN, MIN_SPEED);
	softPwmWrite(IN2_PIN, NORM_SPEED);
	softPwmWrite(IN3_PIN, MIN_SPEED);
	softPwmWrite(IN4_PIN, NORM_SPEED);

	//printf("Go Backward \n");
}

/**
 *@brief Function that turns left with TURN_SPEED fed to right wheels
 *@param none
 *@return none
 */
void MotorControl::goLeftSwing(void)
{
	softPwmWrite(IN1_PIN, MIN_SPEED);
	softPwmWrite(IN2_PIN, MIN_SPEED);
	softPwmWrite(IN3_PIN, TURN_SPEED);
	softPwmWrite(IN4_PIN, MIN_SPEED);
	//printf("LeftSwing\n");  
}
/**
 *@brief Function that turns right with TURN_SPEED fed to left wheels
 *@param none
 *@return none
 */
void MotorControl::goRightSwing(void)
{
	softPwmWrite(IN1_PIN, TURN_SPEED);
	softPwmWrite(IN2_PIN, MIN_SPEED);
	softPwmWrite(IN3_PIN, MIN_SPEED);
	softPwmWrite(IN4_PIN, MIN_SPEED);
	//printf("RightSwing\n");     
}

/**
 *@brief Function that rotates left with TURN_SPEED fed to right wheels reversely and TURN_SPEED fed to left wheels
 *@param none
 *@return none
 */
void MotorControl::goLeftPoint(void)
{
	softPwmWrite(IN1_PIN, MIN_SPEED);
	softPwmWrite(IN2_PIN, TURN_SPEED);
	softPwmWrite(IN3_PIN, TURN_SPEED);
	softPwmWrite(IN4_PIN, MIN_SPEED);
	//printf("LeftPoint\n"); 
}

/**
 *@brief Function that rotates left with NORM_SPEED fed to left wheels reversely and NORM_SPEED fed to right wheels
 *@param none
 *@return none
 */
void MotorControl::goRightPoint(void)
{
	softPwmWrite(IN1_PIN, TURN_SPEED);
	softPwmWrite(IN2_PIN, MIN_SPEED);
	softPwmWrite(IN3_PIN, MIN_SPEED);
	softPwmWrite(IN4_PIN, TURN_SPEED);
	//printf("RightPoint\n");     
}

/**
 *@brief Function that turns left with leftVal fed to left wheels and rightVal fed to right wheels
 *@param leftVal, rightVal
 *@return none
 */
void MotorControl::goCurveTurn(int leftVal, int rightVal)
{
	softPwmWrite(IN1_PIN, leftVal);
	softPwmWrite(IN2_PIN, MIN_SPEED);
	softPwmWrite(IN3_PIN, rightVal);
	softPwmWrite(IN4_PIN, MIN_SPEED);
	//printf("Curveturn: left val: %d, right val: %d\n", leftVal, rightVal);
}

/**
 *@brief Function that stops the car
 *@param none
 *@return none
 */
void MotorControl::stop(void)
{
	softPwmWrite(IN1_PIN, MIN_SPEED);
	softPwmWrite(IN2_PIN, MIN_SPEED);
	softPwmWrite(IN3_PIN, MIN_SPEED);
	softPwmWrite(IN4_PIN, MIN_SPEED);
	//printf("Stop Car\n");  
}

/**
 *@brief Function that sets NORM_SPEED value
 *@param speed
 *@return none
 */
void MotorControl::setSpeed(int speed)
{
	this->NORM_SPEED = speed;
}

/**
 *@brief Function that returns NORM_SPEED value
 *@param speed
 *@return none
 */
int MotorControl::getSpeed()
{
	return this->NORM_SPEED;
}

/**
 *@brief Function that sets TURN_SPEED value
 *@param speed
 *@return none
 */
void MotorControl::setTurnSpeed(int speed)
{
	this->TURN_SPEED = speed;
}


