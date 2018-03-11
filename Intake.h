/*
 * Intake.h
 *
 *  Created on: Jan 24, 2018
 *      Author: Zach
 */

#ifndef INTAKE_H_
#define INTAKE_H_

#include "WPILib.h"

class Intake {
public:
	Intake(uint IntakeMotor1, uint IntakeMotor2);
	~Intake();

	void In();
	void Out();
	void Stop();

private:
	const float Motor_Speed_Intake  		=   1.0;
	const float Motor_Speed_Outtake 		= -0.65;
	const float Motor_Speed_Stop    		=   0.0;
	Spark			*MotorIntake1;
	Spark 			*MotorIntake2;
};

#endif /* INTAKE_H_ */
