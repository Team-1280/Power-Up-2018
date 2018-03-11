
#include "WPILib.h"
#include "../Climber.h"

Climber::Climber(uint climbMotor1Ch, uint climbMotor2Ch)
{
	pClimberMotor1     = new Spark(climbMotor1Ch);
	pClimberMotor2     = new Spark(climbMotor2Ch);

	StopClimber();
}

Climber::~Climber()
{
}

void  Climber::Climb()
{
	pClimberMotor1->Set(MOTOR_SPEED_CLIMB);
	pClimberMotor2->Set(MOTOR_SPEED_CLIMB);
	return;
}

void  Climber::Lower()
{
	pClimberMotor1->Set(MOTOR_SPEED_LOWER);
	pClimberMotor2->Set(MOTOR_SPEED_LOWER);
	return;
}

void Climber::StopClimber()
{
	pClimberMotor1->Set(ALL_STOP);
	pClimberMotor2->Set(ALL_STOP);

	return;
}
