#include "WPILib.h"
#include "Task.h"
#include "Notifier.h"
#include "Timer.h"

class BellRinger
{
	SpeedController& RingerMotor;
	Timer RingTime;
	bool isRinging;
public:
	BellRinger(SpeedController& motor)
	: RingerMotor(motor)
	, RingTime()
	, isRinging(false)
	{
		RingTime.Start();
	}
	
	void ProcessButton(bool shouldRing, bool shouldReverse)
	{
		if (shouldRing)
		{
			if(!isRinging)
			{
				RingTime.Reset();
				RingerMotor.Set(0.5);
				isRinging = true;
			}
			else
			{
				if(RingTime.HasPeriodPassed(0.2))
				{
					RingerMotor.Set(-RingerMotor.Get());
					RingTime.Reset();
				}
			}
		}
		else
		{
			isRinging = false;
			if(shouldReverse)
			{
				RingerMotor.Set(-.5);
			}
			else 
			{
				RingerMotor.Set(0);
			}
		}
	}

private:

};

class Catapult : public PIDOutput
{
	
CANJaguar & Left1;
CANJaguar & Left2;
CANJaguar & Right1;
CANJaguar & Right2;
bool isLaunchOverride;

public:
	Catapult(CANJaguar & l1, CANJaguar & l2, CANJaguar & r1, CANJaguar & r2)
	:Left1(l1),
	 Left2(l2),
	 Right1(r1),
	 Right2(r2),
	 isLaunchOverride(false)
	{
		Left1.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		Left2.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		Right1.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		Right2.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	}
	
	void Set (float speed)
	{
		SmartDashboard::PutNumber("CatapultSpeed", speed);
		Left1.Set(speed);
		Left2.Set(speed);
		Right1.Set(-speed);
		Right2.Set(-speed);
	}
	
	void Launch(float speed, float time)
	{
		isLaunchOverride = true;
		Set(-speed);
		Wait(time);
		Set(0);
		isLaunchOverride = false;
	}
	
	void PIDWrite(float speed)
	{
		if(!isLaunchOverride)
		{
			Set(-speed);
		}
	}
};

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	Victor LeftDrive;
	Victor RightDrive;
	CANJaguar CatapultDriveLeft1;
	CANJaguar CatapultDriveLeft2;
	CANJaguar CatapultDriveRight3;
	CANJaguar CatapultDriveRight4;
	Talon RollerDrive;
	Solenoid BackboardOut;
	Solenoid BackboardIn;
	Solenoid RollerDown;
	Solenoid RollerUp;
	Solenoid DefenceUp;
	Solenoid DefenceDown;
	Solenoid ShiftUp;
	Solenoid ShiftDown;
	
	Compressor Comp;
	
	RobotDrive myRobot; // robot drive system
	Catapult Thrower;
	Joystick DriveStick; // only joystick
	Joystick LaunchStick;
	
	AnalogChannel ThrowingPotent;
	PIDController ThrowerPID;
	
	bool RollerEnabled;
	bool AutoHasLaunched;
		
	float LaunchSpeed;
	float LaunchTime;
	float CarryPosition;
	float StowPosition;
	
public:
	RobotDemo(void):
		LeftDrive (1),
		RightDrive (2),
		CatapultDriveLeft1 (4),
		CatapultDriveLeft2 (5),
		CatapultDriveRight3 (6),
		CatapultDriveRight4 (7),
		RollerDrive (3),
		BackboardOut(8),
		BackboardIn(7),
		RollerDown(1),
		RollerUp(2),
		DefenceUp(6),
		DefenceDown(5),
		ShiftUp(3),
		ShiftDown(4),
		
		Comp(1,8),
		
		myRobot(LeftDrive, RightDrive),
		Thrower(CatapultDriveLeft1, CatapultDriveLeft2, CatapultDriveRight3, CatapultDriveRight4),
		DriveStick(1),
		LaunchStick(2),
		
		ThrowingPotent(1),
		ThrowerPID(-.5,-.01,0,&ThrowingPotent, &Thrower),
		
		RollerEnabled(false),
		AutoHasLaunched(false),
		
		LaunchSpeed(1),
		LaunchTime(.3),
		CarryPosition(2.3),
		StowPosition(1.9)



	{
		LiveWindow::GetInstance()->AddSensor("Thrower","Potentiometer",&ThrowingPotent);
		LiveWindow::GetInstance()->AddActuator("Thrower","PIDControl",&ThrowerPID);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Left1", &CatapultDriveLeft1);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Left2", &CatapultDriveLeft2);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Right1", &CatapultDriveRight3);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Right2", &CatapultDriveRight4);
		
		InitializeVariablesFromParams();
		
		ThrowingPotent.SetVoltageForPID(true);

		ThrowerPID.SetOutputRange(-0.5,0.5);
		myRobot.SetExpiration(0.1);
	}

	void InitializeVariablesFromParams()
	{
		Preferences* prefs = Preferences::GetInstance();
		string value = "";
		value = prefs->GetString("LaunchSpeed", ".9");//("LaunchSpeed", 1.0);
		LaunchSpeed = ::atof(value.c_str());
		LaunchTime = prefs->GetFloat("LaunchTime", 0.3);
		CarryPosition = prefs->GetFloat("CarryPosition", 2.15);
		StowPosition = prefs->GetFloat("StowPosition", 1.9);
		
		float throwP = prefs->GetFloat("ThrowP", -0.75);
		float throwI = prefs->GetFloat("ThrowI", -0.05);
		float throwD = prefs->GetFloat("ThrowD", 0);
		ThrowerPID.SetPID(throwP, throwI, throwD);
	}
	
	void SetPneumaticsSafe()
	{
		BackboardIn.Set(true);
		RollerUp.Set(true);
		DefenceUp.Set(true);
		ShiftDown.Set(true);
		Wait(0.02);
		BackboardIn.Set(false);
		RollerUp.Set(false);
		DefenceUp.Set(false);
		ShiftDown.Set(false);
	}
	
	void Autonomous(void)
	{
//		while(IsAutonomous() && !AutoHasLaunched)
//		{
//			//Add code to set AutoHasLaunched before the if statement where it gets checked or that code will never get called.
//			
//			
//			if(targetDetected)
//			{
//				Thrower.Set(SmartDashboard::GetNumber("AutonomousLaunchPowah"));
				//TODO If 7 seconds time out of Autonomous, launch anyway
				//TODO Make a check if launched function before drive
				//TODO Add code to drive robot forward certain distance
//				AutoHasLaunched = true;
//				break;
//			}
//		}
//		
	}
	
	void ProcessDriveStick()
    {
        myRobot.ArcadeDrive(DriveStick.GetY(), -DriveStick.GetX());
        ShiftUp.Set(DriveStick.GetRawButton(3));
        ShiftDown.Set(DriveStick.GetRawButton(2));
        BackboardOut.Set(DriveStick.GetRawButton(8) || DriveStick.GetRawButton(4));
        BackboardIn.Set(DriveStick.GetRawButton(9) || DriveStick.GetRawButton(5));
        DefenceUp.Set(DriveStick.GetRawButton(6) || DriveStick.GetRawButton(11));
        DefenceDown.Set(DriveStick.GetRawButton(7) || DriveStick.GetRawButton(10));
    }

	void ProcessLaunchStick()
	{
		float catapultSpeed = LaunchStick.GetY();
//		SmartDashboard::PutNumber("CatapultCommandedSpeed", catapultSpeed);
		if (LaunchStick.GetRawButton(1))
		{
			Thrower.Set(catapultSpeed);
		}
		else
		{
			Thrower.Set(0);
		}
		
		//float adjustSpeed = SmartDashboard::GetNumber("catapultAdjustSpeed");
		//SmartDashboard::PutNumber("tstReceiveValue", adjustSpeed);
		float adjustSpeed = 0;
		if(LaunchStick.GetRawButton(4))
		{
			Thrower.Set(adjustSpeed);
		}
		if(LaunchStick.GetRawButton(5))
		{
			Thrower.Set(-adjustSpeed);
		}
				
		RollerUp.Set(LaunchStick.GetRawButton(3));
		RollerDown.Set(LaunchStick.GetRawButton(2));
		float rollerSpeed = LaunchStick.GetZ();
		if(LaunchStick.GetRawButton(6) || LaunchStick.GetRawButton(11))
		{
			RollerEnabled = true;
		}
		if(LaunchStick.GetRawButton(7) || LaunchStick.GetRawButton(10))
		{
			RollerEnabled = false;
		}
		if(RollerEnabled)
		{
			RollerDrive.Set(rollerSpeed);
		}
		else
		{
			RollerDrive.Set(0);
		}
		if(LaunchStick.GetRawButton(8) || LaunchStick.GetRawButton(9))
		{
			
		}
//		SmartDashboard::PutNumber("RollerSpeed *derp*", rollerSpeed);
	}
	
	void ProcessLaunchStickOther()
	{
		//ThrowerControl		
		if(LaunchStick.GetRawButton(5))
		{
			ThrowerPID.Reset();
			ThrowerPID.SetSetpoint(CarryPosition);
			ThrowerPID.Enable();
		}
		if(LaunchStick.GetRawButton(4))
		{
			ThrowerPID.Reset();			
			ThrowerPID.SetSetpoint(StowPosition);
			ThrowerPID.Enable();
		}
		if(LaunchStick.GetRawButton(7) || LaunchStick.GetRawButton(10))
		{
			ThrowerPID.Disable();
		}
		if(LaunchStick.GetRawButton(1))
		{
			ThrowerPID.Disable();
			Thrower.Launch(LaunchSpeed, LaunchTime);
		}
		
		if(LaunchStick.GetRawButton(11))
		{
			Thrower.Set(0.1);
		}
		else if(LaunchStick.GetRawButton(6))
		{
			Thrower.Set(-0.1);
		}
		
		//RollerControl
		RollerUp.Set(LaunchStick.GetRawButton(3));
		RollerDown.Set(LaunchStick.GetRawButton(2));
		
		float rollerSpeed = LaunchStick.GetY();
		if(rollerSpeed < 0.1 && rollerSpeed > -0.1)
		{
			rollerSpeed = 0;
		}
		RollerDrive.Set(rollerSpeed);		
	}
	
    void OperatorControl(void)
    {
		ThrowerPID.Reset();
    	Comp.Start();
        InitializeVariablesFromParams();
        
		SmartDashboard::PutNumber("ValueIGotLaunchSpeed", LaunchSpeed);
		SmartDashboard::PutNumber("ValueIGotLaunchTime", LaunchTime);
		SmartDashboard::PutNumber("ValueIGotCarryPosition", CarryPosition);
		SmartDashboard::PutNumber("ValueIGotStowPosition", StowPosition);
		SmartDashboard::PutNumber("ValueIGotP", ThrowerPID.GetP());
		SmartDashboard::PutNumber("ValueIGotI", ThrowerPID.GetI());
		

        myRobot.SetSafetyEnabled(true);
        while(IsOperatorControl() && IsEnabled())
        {
            ProcessDriveStick();
			
            //ProcessLaunchStick();
            ProcessLaunchStickOther();
            
            SmartDashboard::PutNumber("PotentiometerValue" , ThrowingPotent.GetVoltage());
			Wait(0.005);
		}
		Comp.Stop();
	}
	
	void Test(void)
	{
		SetPneumaticsSafe();
		Comp.Start();
		while (IsTest() && IsEnabled())
		{
			LiveWindow::GetInstance()->Run();
			Wait(0.1);
		}
		Comp.Stop();
	}
};

START_ROBOT_CLASS(RobotDemo);
