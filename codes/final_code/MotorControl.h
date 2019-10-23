#ifndef _MotorControl1_H__
#define _MotorControl1_H__


/**
 *@brief Definition of the MotorControl class. It contains all the functions and variables depicted in the
 *@brief It contains functions for basic motor control three type of turns
 *@brief same image with the plotted lane.
 */
class MotorControl
{
public:
    // Constructor:
    MotorControl();
	
	// Functions:
	void init(void); // Intializes motor pins
	void goForward(void); // Goes forward with NORM_SPEED
	void goBack(void); // Goes backward with NORM_SPEED
	void goLeftSwing(void); // Turns left with TURN_SPEED
	void goRightSwing(void); // Turns right with TURN_SPEED
	void goLeftPoint(void); // Performs point turn to the right with TURN_SPEED
	void goRightPoint(void); // Performs point turn to the left with TURN_SPEED
	void goCurveTurn(int, int); // Performs curve turn by fedding specific values to left and right wheels
	void stop(void); // Stops car
	void setSpeed(int); // Sets NORM_SPEED value
	int getSpeed(); // Gets NORM_SPEED value
	void setTurnSpeed(int); // Sets TURN_SPEED value

	// Destructor:
	~MotorControl();

private:	
	int IN1_PIN; // Left pin1
	int IN2_PIN; // Left pin2
	int IN3_PIN; // Right pin1
	int IN4_PIN; // Right pin2
	int NORM_SPEED;	// Normal speed
	int TURN_SPEED; // Speed used when turning
};

#endif // _MotorControl_H__
