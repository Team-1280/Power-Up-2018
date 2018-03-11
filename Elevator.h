/*
 * Elevator.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Zach
 */
#include "WPILib.h"

#ifndef ELEVATOR_H_
#define ELEVATOR_H_

class Elevator {
public:
	Elevator(uint Elevator_Motor1_CH, uint Elevator_Motor2_CH);
	~Elevator();
	void Up1(float speed);
	void Down1(float speed);
	void Up2(float speed);
	void Down2(float speed);
//	void Down(float speed);
	void Stop();
	void Stop2();

private:
	const float Motor_Speed_Up  	=  1.0;
	const float Motor_Speed_Down	= -1.0;
	const int Motor_Speed_Stop		=  0.0;
	Spark *pElvMotor1;
    Spark *pElvMotor2;
};

#endif /* ELEVATOR_H_ */
