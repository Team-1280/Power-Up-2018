//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2018 POWER UP ROBOT CODE
// Created on: Jan 24, 2018
//      Authors: Zach, Tyler
// Advice to all future 1280 progrSammers: Don't take advice from the angry robot lady
//------------------------------------------------------------------------------

#include <iostream>
#include <string>
#include <cmath>

#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DriverStation.h>
#include "../Climber.h"
#include "../Intake.h"
#include "../Elevator.h"

#define CONSOLE

class PowerUpRobot : public IterativeRobot {

	public:

		 PowerUpRobot();
		~PowerUpRobot();

		void   RobotInit();
		void   DisabledInit();
		void   AutonomousInit();
		void   TeleopInit();
		void   DisabledPeriodic();
		void   AutonomousPeriodic();
		void   TeleopPeriodic();

	private:

//---------------------------------------------------------
//Robot Constants
//---------------------------------------------------------

		static const uint JS_PORT_LEFT         		    =  0;
		static const uint JS_PORT_RIGHT			    =  1;
		static const uint CCI_PORT1        	  	    =  2;

		// Joystick Button (right joystick)
		static const uint CAMERA_LIGHT_SW_CH       	    =  1;

		// Driver Station CCI1 Channels (Uses joystick button references)
		static const uint Down_SW_CH		 	       =  6;
		static const uint Climber_SW_CH			       =  5;
		static const uint AIROUT_SW_CH 			       =  2;
		static const uint Elevator1_SW_CH		       =  1;
		static const uint Elevator2_SW_CH		       =  4;
		static const uint Intake_SW_CH		  	       =  3;
		static const uint Outtake_SW_CH		   	       =  2;

		// roboRio PWM Channels (PWM = Pulse width modulation)
		static const uint RIGHT_FRONT_MOTOR_CH	  	   =  0;
		static const uint RIGHT_REAR_MOTOR_CH      	   =  1;
		static const uint LEFT_FRONT_MOTOR_CH	           =  3;
		static const uint LEFT_REAR_MOTOR_CH	   	   =  2;
		static const uint LEFT_INTAKE_MOTOR_CH	  	   =  6;
		static const uint RIGHT_INTAKE_MOTOR_CH	  	   =  7;
		static const uint CLIMBER_MOTOR1_CH	  	   =  4;
		static const uint CLIMBER_MOTOR2_CH     	   =  5;
		static const uint ELEVATOR_MOTOR1_CH 	           =  8;
		static const uint ELEVATOR_MOTOR2_CH	           =  9;

		// roboRio Solenoid Channels
		static const uint COMPRESSOR_CH 	           =  0;
		static const uint SOLENOID_CH_1A	           =  0;
		static const uint SOLENOID_CH_2A	           =  1;
		static const uint SOLENOID_CH_1B                   =  2;
		static const uint SOLENOID_CH_2B	  	   =  3;
		static const uint SOLENOID_CH_1C	           =  4;
		static const uint SOLENOID_CH_2C	      	   =  5;

		// Solenoid states:
		static const int SOLENOID_REVERSE = frc::DoubleSolenoid::Value::kReverse;
		static const int SOLENOID_FORWARD = frc::DoubleSolenoid::Value::kForward;

//---------------------------------------------------------
//AUTO Variables
//---------------------------------------------------------

		// Auto Drive speed
		const float Speed_Drive_Auto    	           =  0.5;
		const float Speed_Elevator_Auto		           =  0.5;

		//Starting positions
		std::string gameData;
		int Starting_Position;

//---------------------------------------------------------
//Teleop 	objects & pointers & varaibles
//---------------------------------------------------------

		//Joystick pointers
		Joystick		 *pDriveStickLeft   = nullptr;
		Joystick		 *pDriveStickRight  = nullptr;
		Joystick         *pXBox             = nullptr;

		//joystick button pointers
		JoystickButton 	 	*pClimbButton   = nullptr;
		JoystickButton   	*pDownButton    = nullptr;
		JoystickButton   	*pAirIn         = nullptr;

		//Pointers for robot mechanisms
		RobotDrive		*pDriveTrain = nullptr;
		Climber			*pClimber    = nullptr;
		Intake 			*pIntake     = nullptr;
		Elevator 		*pElevator   = nullptr;
		Compressor 		*pCompressor = nullptr;

		//Objects for double solenoids
		frc::DoubleSolenoid Solenoid1  {SOLENOID_CH_1A, SOLENOID_CH_2A};  //Intake
		frc::DoubleSolenoid Solenoid2  {SOLENOID_CH_1B, SOLENOID_CH_2B};  //Elevator Stage 1 bike stopper
		frc::DoubleSolenoid Solenoid3  {SOLENOID_CH_1C, SOLENOID_CH_2C};  //Intake Lower System

		// variables to track counts/states
		uint   loopCount;
		uint   DelayCounter;
		bool   IntakePiston;
		bool   TempLastIntake;

		//Drive Train speeds
		float  rightDriveSpeed;
		float  leftDriveSpeed;

		//Elevator Speeds
		float  LeftY;
		float  RightY;

		//Intake triggers
		float  LeftTrigger;
		float  RightTrigger;

		// Tolerance values - Joystick
		const float Joystick_Tolerance	  =  0.3;


//---------------------------------------------------------
//Method declarations
//---------------------------------------------------------

		// Driver station and robot input gathering methods
		void   GetDriverStationInput();
		void   ShowDSValues();

		// Autonomous mode methods
		void   RunClimber();
		void   RunIntake();
		void   RunElevator();
		void   AutoCubeSide();
		void   AutoCubeCenter();
		void   AutoSwitch();
		void   SolenoidUpdate();
		void   AirIn();
		void   AirOut();
		void   Stage1Stop();
		void   Stage2Stop();
		void   Stage1Start();
		void   Stage2Start();
		void   StageUpdate();
		void   Delay();
		bool   FMSFetch();
		bool   RightTriggerPressed();
		bool   LeftTriggerPressed();
};

START_ROBOT_CLASS(PowerUpRobot)

PowerUpRobot::PowerUpRobot()
{
//joysticks & XBox
	pDriveStickLeft		   = new Joystick(JS_PORT_LEFT);
	pDriveStickRight	   = new Joystick(JS_PORT_RIGHT);
	pXBox               	   = new Joystick(CCI_PORT1);

	pClimbButton 		   = new JoystickButton(pXBox,Down_SW_CH);
	pDownButton 		   = new JoystickButton(pXBox,Climber_SW_CH);
	pAirIn		           = new JoystickButton(pXBox,AIROUT_SW_CH);
	pCompressor 		   = new Compressor(COMPRESSOR_CH);

//Mechanism pointer initialization
	pDriveTrain		       = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,RIGHT_FRONT_MOTOR_CH, RIGHT_REAR_MOTOR_CH);
	pClimber		       = new Climber(CLIMBER_MOTOR1_CH);
	pIntake 		       = new Intake(LEFT_INTAKE_MOTOR_CH, RIGHT_INTAKE_MOTOR_CH);
	pElevator 		       = new Elevator(ELEVATOR_MOTOR1_CH, ELEVATOR_MOTOR2_CH);

//Variable initializations
	loopCount      		   = 0;
	DelayCounter		   = 0;
	IntakePiston           = true;
	TempLastIntake 	       = false;

	rightDriveSpeed        = 0.0;
	leftDriveSpeed    	   = 0.0;

	LeftY		           = 0.0;
	RightY		           = 0.0;
	LeftTrigger  		   = 0.0;
	RightTrigger		   = 0.0;

	Starting_Position      = 1;
}

PowerUpRobot::~PowerUpRobot()
{
}

void PowerUpRobot::RobotInit()
{
#ifdef CONSOLE
	SmartDashboard::init();
#endif
}

void PowerUpRobot::DisabledInit()
{
	loopCount     = 0;
	AirOut();

}

void PowerUpRobot::AutonomousInit()
{
	loopCount      = 0;
	Starting_Position = frc::DriverStation::GetInstance().GetLocation();
	gameData= frc::DriverStation::GetInstance().GetGameSpecificMessage();
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);

}

void PowerUpRobot::TeleopInit()
{
	loopCount      = 0;
	DelayCounter   = 0;
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	pCompressor->SetClosedLoopControl(true);

}

void PowerUpRobot::DisabledPeriodic()
{
	loopCount++;
	AirOut();

#ifdef CONSOLE
#endif
}

void PowerUpRobot::AutonomousPeriodic()
{
	//waiting for fms values to be returned
	if(FMSFetch()){
	loopCount++;
	DelayCounter++;
	SmartDashboard::PutNumber("LoopCount",loopCount);
	}
	else{
		Starting_Position = frc::DriverStation::GetInstance().GetLocation();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}

}

void PowerUpRobot::TeleopPeriodic()
{
//update expressions
	loopCount++;
	DelayCounter++;
	GetDriverStationInput();
	pDriveTrain->TankDrive(leftDriveSpeed,rightDriveSpeed);

    RunClimber();
    RunIntake();
    RunElevator();
    SolenoidUpdate();

}

void PowerUpRobot::GetDriverStationInput()
{
//Joystick Input
	rightDriveSpeed	    =  -1*pDriveStickRight->GetY();//Inverted direction to correct mistake
	leftDriveSpeed	    =  -1*pDriveStickLeft->GetY();//Inverted direction to correct mistake

//Xbox Input
	LeftY               = pXBox->GetRawAxis(Elevator1_SW_CH);
	RightY              = pXBox->GetRawAxis(Elevator2_SW_CH);
	LeftTrigger         = pXBox->GetRawAxis(Intake_SW_CH);
	RightTrigger        = pXBox->GetRawAxis(Outtake_SW_CH);

#ifdef CONSOLE
    ShowDSValues();
#endif

}
#ifdef CONSOLE

void PowerUpRobot::ShowDSValues()
{
	// Show the values for driver station inputs
	SmartDashboard::PutNumber ("Joystick Right",pDriveStickRight->GetY());
	SmartDashboard::PutNumber ("Joystick Left",pDriveStickLeft->GetY());
	//	SmartDashboard::PutBoolean("DS Intake Robot",pIntakeButton->Get());
	SmartDashboard::PutBoolean("DS Air In     ",pAirIn->Get());

}
#endif

void PowerUpRobot::RunClimber()
{
	if(pClimbButton->Get()){
		pClimber->Climb();
	}
	else{
		if(pDownButton->Get()){
			pClimber->Lower();
		}
		else{
			pClimber->StopClimber();
		}
	}

}

void PowerUpRobot::RunIntake(){
	bool rightTrigger = RightTrigger > Joystick_Tolerance;
	bool leftTrigger  = LeftTrigger   > Joystick_Tolerance;
	if((rightTrigger || leftTrigger ) && (rightTrigger != leftTrigger)){ // XOR logical operation
		if ( rightTrigger )
		{
			pIntake->In();
		}
		else{
			pIntake->Out();
		}
	}
	else{
		pIntake->Stop();
	}

}

void PowerUpRobot::AirOut(){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);

}

void PowerUpRobot::AirIn(){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);

}


void PowerUpRobot::SolenoidUpdate(){
	if(!TempLastIntake && pAirIn->Get()){
			IntakePiston ? AirOut() : AirIn();
			IntakePiston = !IntakePiston;
			TempLastIntake=true;
	}
	else if(!pAirIn->Get()){//Changed to account for button not being pressed
		TempLastIntake=false;
	}
	/*else{
		TempLastIntake = false;
	}*/
}

void   PowerUpRobot::RunElevator(){
	//tolerance for Xbox-joystick
	if(LeftTriggerPressed()){
		pElevator->Up1(-LeftY);
	}
	else{
		pElevator->Stop();
		Stage1Stop();
	}
	//Stage 2:
	if(RightTriggerPressed())
	{
		pElevator->Up2(-RightY);
	}
	else{
		pElevator->Stop2();
		Stage2Stop();
	}
	StageUpdate();
}

void PowerUpRobot::Stage1Stop(){
	Solenoid2.Set(frc::DoubleSolenoid::Value::kForward);

}

void PowerUpRobot::Stage1Start(){
	Solenoid2.Set(frc::DoubleSolenoid::Value::kReverse);

}

void PowerUpRobot::Stage2Stop(){
	Solenoid3.Set(frc::DoubleSolenoid::Value::kForward);

}

void PowerUpRobot::Stage2Start(){
	Solenoid3.Set(frc::DoubleSolenoid::Value::kReverse);
}

void PowerUpRobot::StageUpdate(){
	if(LeftTriggerPressed()){
		Stage2Start();
	}
	if(RightTriggerPressed()){
		Stage1Start();
	}
}

bool PowerUpRobot::FMSFetch(){
	return (gameData=="L"||gameData=="R")&&(Starting_Position==1||Starting_Position==2||Starting_Position==3);
}

bool PowerUpRobot::LeftTriggerPressed(){
	return abs(LeftY) > Joystick_Tolerance;
}

bool PowerUpRobot::RightTriggerPressed(){
	return abs(RightY) > Joystick_Tolerance;
}
