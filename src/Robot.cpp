//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2018 POWER UP ROBOT CODE
// Created on: Jan 24, 2018
//      Authors: Zach, Tyler
// Advice to all future 1280 progrSammers: Don't take advice from the angry robot lady
// Don't toucha my spaghett
// kill yourself
//------------------------------------------------------------------------------

#include <iostream>
#include <string>
#include "WPILib.h"
#include <cmath>

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

		static const uint JS_PORT_LEFT             =  0;
		static const uint JS_PORT_RIGHT			   =  1;
		static const uint CCI_PORT1        	       =  2;

		// Joystick Button (right joystick)
		static const uint CAMERA_LIGHT_SW_CH       =  1;

		// Driver Station CCI1 Channels (Uses joystick button references)
		static const uint Down_SW_CH		 	   =  6;
		static const uint Climber_SW_CH			   =  5;
		static const uint AIROUT_SW_CH 			   =  2;
		static const uint Elevator1_SW_CH		   =  1;
		static const uint Elevator2_SW_CH		   =  5;
		static const uint Intake_SW_CH		  	   =  3;
		static const uint Outtake_SW_CH		   	   =  2;

		// roboRio PWM Channels (PWM = Pulse width modulation)
		static const uint RIGHT_FRONT_MOTOR_CH	   =  0;
		static const uint RIGHT_REAR_MOTOR_CH      =  1;
		static const uint LEFT_FRONT_MOTOR_CH	   =  3;
		static const uint LEFT_REAR_MOTOR_CH	   =  2;
		static const uint LEFT_INTAKE_MOTOR_CH	   =  6;
		static const uint RIGHT_INTAKE_MOTOR_CH	   =  7;
		static const uint CLIMBER_MOTOR1_CH		   =  4;
		static const uint CLIMBER_MOTOR2_CH		   =  5;
		static const uint ELEVATOR_MOTOR1_CH 	   =  8;
		static const uint ELEVATOR_MOTOR2_CH	   =  9;

		// roboRio Solenoid Channels
		static const uint Compressor_CH 		   =  0;
		static const uint Solenoid1a_CH			   =  0;
		static const uint Solenoid2a_CH	   		   =  1;
		static const uint Solenoid1b_CH			   =  2;
		static const uint Solenoid2b_CH	   		   =  3;
		static const uint Solenoid1c_CH			   =  4;
		static const uint Solenoid2c_CH	   		   =  5;

//---------------------------------------------------------
//AUTO Variables
//---------------------------------------------------------

		// Auto Drive speed
		const float Speed_Drive_Auto    	       =  0.5;
		const float Speed_Elevator_Auto		       =  0.5;

		// Auto Drive Time
		static const uint  Time_Foward_ToNull      =  500;
		static const uint  Time_Turn_Center	       =  210;
		static const uint  Time_Turn_Side          =  230;
		static const uint  Time_Foward_Side1	   =  14/27 * Time_Foward_ToNull;
		static const uint  Time_Foward_Side2	   =  Time_Turn_Side   + Time_Foward_Side1  + 1.1/27  * Time_Foward_ToNull;
		static const uint  Time_Foward_Center      =  Time_Turn_Center                      + 10.9/27 * Time_Foward_ToNull;
		static const uint  Time_Elevator		   =  250;

		//Starting positions
		std::string gameData;
		int Starting_Position;

//---------------------------------------------------------
//Teleop 	objects & pointers & varaibles
//---------------------------------------------------------

		//Joystick pointers
		Joystick		 *pDriveStickLeft;
		Joystick		 *pDriveStickRight;
		Joystick         *pXBox;

		//joystick button pointers
		JoystickButton 	 *pClimbButton;
		JoystickButton   *pDownButton;
		JoystickButton   *pAirIn;

		//Pointers for robot mechanisms
		RobotDrive		*pDriveTrain;
		Climber			*pClimber;
		Intake 			*pIntake;
		Elevator 		*pElevator;
		Compressor 		*pCompressor;

		//Objects for double solenoids
		frc::DoubleSolenoid Solenoid1  {Solenoid1a_CH, Solenoid2a_CH};  //Intake
		frc::DoubleSolenoid Solenoid2  {Solenoid1b_CH, Solenoid2b_CH};  //Elevator Stage 1 bike stopper
		frc::DoubleSolenoid Solenoid3  {Solenoid1c_CH, Solenoid2c_CH};  //Elevator Stage 2 bike stopper

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
};

START_ROBOT_CLASS(PowerUpRobot)

PowerUpRobot::PowerUpRobot()
{
//joysticks & XBox
	pDriveStickLeft		   = new Joystick(JS_PORT_LEFT);
	pDriveStickRight	   = new Joystick(JS_PORT_RIGHT);
	pXBox                  = new Joystick(CCI_PORT1);

	pClimbButton 		   = new JoystickButton(pXBox,Down_SW_CH);
	pDownButton 		   = new JoystickButton(pXBox,Climber_SW_CH);
	pAirIn				   = new JoystickButton(pXBox,AIROUT_SW_CH);
	pCompressor 		   = new Compressor(Compressor_CH);

//Mechanism pointer initialization
	pDriveTrain		       = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,RIGHT_FRONT_MOTOR_CH, RIGHT_REAR_MOTOR_CH);
	pClimber		       = new Climber(CLIMBER_MOTOR1_CH, CLIMBER_MOTOR2_CH);
	pIntake 			   = new Intake(LEFT_INTAKE_MOTOR_CH, RIGHT_INTAKE_MOTOR_CH);
	pElevator 			   = new Elevator(ELEVATOR_MOTOR1_CH, ELEVATOR_MOTOR2_CH);

//Variable initializations
	loopCount      		   = 0;
	DelayCounter		   = 0;
	IntakePiston           = true;
	TempLastIntake 		   = false;

	rightDriveSpeed        = 0.0;
	leftDriveSpeed         = 0.0;

	LeftY				   = 0.0;
	RightY				   = 0.0;
	LeftTrigger  		   = 0.0;
	RightTrigger		   = 0.0;

	Starting_Position      = 1;
	return;
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
	loopCount  = 0;
	AirOut();
	return;
}

void PowerUpRobot::AutonomousInit()
{
	loopCount      = 0;
	Starting_Position = frc::DriverStation::GetInstance().GetLocation();
	gameData= frc::DriverStation::GetInstance().GetGameSpecificMessage();
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}

void PowerUpRobot::TeleopInit()
{
	loopCount      = 0;
	DelayCounter   = 0;
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	pCompressor->SetClosedLoopControl(true);
	return;
}

void PowerUpRobot::DisabledPeriodic()
{
	loopCount++;
	AirOut();
#ifdef CONSOLE
#endif
	return;
}

void PowerUpRobot::AutonomousPeriodic()
{
	if(FMSFetch()){
	loopCount++;
	DelayCounter++;
	AutoSwitch();
	SmartDashboard::PutNumber("LoopCount",loopCount);
	}
	else{
		Starting_Position = frc::DriverStation::GetInstance().GetLocation();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}
	return;
}
/*
// if we don't have enough time to test auto
if(Starting_Position==1||Starting_Position==3){
//go to null
	if (loopCount < Time_Foward_ToNull) {//insert time
			pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
	}
	else{
	}
	}
}
else{
//do nothing
}
*/

void PowerUpRobot::TeleopPeriodic()
{
//update expressions
	loopCount++;
	DelayCounter++;
	GetDriverStationInput();

	pDriveTrain->TankDrive(-leftDriveSpeed,-rightDriveSpeed);
    RunClimber();
    RunIntake();
    RunElevator();
    SolenoidUpdate();
	return;
}

void PowerUpRobot::GetDriverStationInput()
{
//Joystick Input
	rightDriveSpeed		=  pDriveStickRight->GetY();
	leftDriveSpeed		=  pDriveStickLeft->GetY();

//Xbox Input
	LeftY               = pXBox->GetRawAxis(Elevator1_SW_CH);
	RightY              = pXBox->GetRawAxis(Elevator2_SW_CH);
	LeftTrigger         = pXBox->GetRawAxis(Intake_SW_CH);
	RightTrigger        = pXBox->GetRawAxis(Outtake_SW_CH);

#ifdef CONSOLE
    ShowDSValues();
#endif
	return;
}
#ifdef CONSOLE

void PowerUpRobot::ShowDSValues()
{
	// Show the values for driver station inputs
	SmartDashboard::PutNumber ("Joystick Right",pDriveStickRight->GetY());
	SmartDashboard::PutNumber ("Joystick Left",pDriveStickLeft->GetY());
	//	SmartDashboard::PutBoolean("DS Intake Robot",pIntakeButton->Get());
	SmartDashboard::PutBoolean("DS Air In     ",pAirIn->Get());
	return;
}
#endif

void PowerUpRobot::AutoSwitch(){
	switch(Starting_Position){
				case 1:					//LEFT Side
					if(gameData[0]=='L'){
						//put cube on switch
						AutoCubeSide();
					}
					else if(gameData[0]=='R'){
						// go to null
							if (loopCount < Time_Foward_ToNull) {
								pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
							}
							else{

								}
								}
								else{

								}
								break;
				case 2:					//CENTER field
					if(gameData[0]=='L'){
						// go left
						if (loopCount < Time_Turn_Center) {
							pDriveTrain->TankDrive(-Speed_Drive_Auto,Speed_Drive_Auto);
						}
						else if(loopCount < Time_Foward_Center){
						}
						else{
						AutoCubeCenter();
						}
						}
						else if(gameData[0]=='R'){
							// go right
							if (loopCount < Time_Turn_Center) {
								pDriveTrain->TankDrive(Speed_Drive_Auto,-Speed_Drive_Auto);
							}
							else if(loopCount < Time_Foward_Center){
								pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
							}
							else{
								AutoCubeCenter();
								}
							}
							else{

							}
							break;
				case 3:					//RIGHT Side
					if(gameData[0]=='L'){
						//go to null
						if (loopCount < Time_Foward_ToNull) {//insert time
							pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
						}
						else{
						}
						}
						else if(gameData[0]=='R'){
							//put cube on switch
							AutoCubeSide();
						}
						else{

						}
						break;
						}
	return;
}

void PowerUpRobot::AutoCubeSide(){
	int Speed_Auto_Turn;
	// assigns direction for turning based on location
	if(Starting_Position==1){
			Speed_Auto_Turn		=	Speed_Drive_Auto;
	}
	else{
			Speed_Auto_Turn		=	-Speed_Drive_Auto;
	}

	if(loopCount<Time_Foward_Side1){
		pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
	}
	else{
	if(loopCount<Time_Turn_Side){
		pDriveTrain->TankDrive(Speed_Auto_Turn,-Speed_Auto_Turn);
	}
	else{
	if(loopCount<Time_Foward_Side2){
		pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
		}
		else{
		if(loopCount<Time_Elevator){
			pElevator->Up2(Speed_Elevator_Auto);
		}
		else{
			pElevator->Stop();
			Stage1Stop();
			Stage2Stop();
			AirOut();
			pIntake->Out();
		}
		}
		}
	}
	return;
}

void PowerUpRobot::AutoCubeCenter(){
	if(loopCount<Time_Elevator){
				Stage1Start();
				Stage2Start();
				pElevator->Up2(Speed_Elevator_Auto);
	}
			else{
				pElevator->Stop();
				Stage1Stop();
				Stage2Stop();
				AirOut();
				pIntake->Out();
			}
	return;
}

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
	return;
}

void PowerUpRobot::RunIntake(){
	if ( RightTrigger > Joystick_Tolerance )
	{
		pIntake->In();
	}
	else
	{
		if ( LeftTrigger > Joystick_Tolerance )
		{
			pIntake->Out();
		}
		else
		{
			pIntake->Stop();
		}
	}
	return;
}

void PowerUpRobot::AirOut(){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::AirIn(){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}


void PowerUpRobot::SolenoidUpdate(){
	if(pAirIn->Get()){
		if(!TempLastIntake){
	if(IntakePiston){
		AirOut();
		IntakePiston=false;
	}
	else{
		AirIn();
		IntakePiston=true;
	}
	TempLastIntake=true;
		}
	}
	else{
		TempLastIntake=false;
	}
	return;
}

void   PowerUpRobot::RunElevator(){
	//tolerance for Xbox-joystick
	if(LeftY > Joystick_Tolerance || LeftY < -Joystick_Tolerance){
			    pElevator->Up1(-LeftY);
		}
	else{
				pElevator->Stop();
				Stage1Stop();
	}
	//Stage 2:
	if(RightY > Joystick_Tolerance || RightY < -Joystick_Tolerance)
	{
				pElevator->Up2(-RightY);
			}
	else{
				pElevator->Stop2();
				Stage2Stop();
		}
	StageUpdate();
return;
}

void PowerUpRobot::Stage1Stop(){
	Solenoid2.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::Stage1Start(){
	Solenoid2.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}

void PowerUpRobot::Stage2Stop(){
	Solenoid3.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::Stage2Start(){
	Solenoid3.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}

void PowerUpRobot::StageUpdate(){
			if(LeftY > Joystick_Tolerance || LeftY < -Joystick_Tolerance){
				Stage2Start();
			}
			if(RightY > Joystick_Tolerance || RightY < -Joystick_Tolerance){
				Stage1Start();
			}
}

bool PowerUpRobot::FMSFetch(){
	if((gameData=="L"||gameData=="R")&&(Starting_Position==1||Starting_Position==2||Starting_Position==3)){
		return true;
	}
	else{

		return false;
	}
}
