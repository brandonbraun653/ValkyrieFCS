#pragma once
#ifndef MOTORS_HPP_
#define MOTORS_HPP_

enum MotorState
{
	MOTOR_STATE_OFF,
	MOTOR_STATE_ARM,
	MOTOR_STATE_IDLE,
	MOTOR_STATE_KILL
};
extern MotorState MotorControllerState;

extern void motorTask(void* argument);


#endif /* MOTORS_HPP_ */