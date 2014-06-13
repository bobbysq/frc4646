#include "WPILib.h"
#include "Task.h"
#include "Notifier.h"
#include "Timer.h"

const float LOW_POT = 3.10;
const float MID_POT = 3.482;
const float HIGH_POT = 4.667;


class Catapult
{
public:
enum CatapultModes
{
	Idle,
	Manual,
	Pickup,
	Carry,
	LaunchHold
};

	Catapult(CANJaguar & l1, CANJaguar & l2, CANJaguar & r1, CANJaguar & r2, AnalogChannel& pot)
	:Left1(l1),
	 Left2(l2),
	 Right1(r1),
	 Right2(r2),
	 Potent(pot),
	 isLaunchOverride(false),
	 Mode(Idle),
	 PickupPosition(LOW_POT),
	 CarryPosition(HIGH_POT),
	 LaunchHoldPosition(MID_POT)
	 
	{
		Potent.SetVoltageForPID(true);
		Left1.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		Left2.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		Right1.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		Right2.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	}
	
	void SetManual(float speed)
	{
		if(Manual == Mode)
		{
			Set(speed);
		}
	}
	
	
	void Launch(float speed, float time)
	{
		isLaunchOverride = true;
		Set(-speed);
		Wait(time);
		Set(0);
		isLaunchOverride = false;
	}
		
	void SetMode(CatapultModes newMode)
	{
		Mode = newMode;
	}
	
	void ProcessMode()
	{
		switch(Mode)
		{
		case Idle:
			Set(0);
			break;
		case Manual:
			//do nothing
			break;
		case Pickup:
			if(IsClawAbovePickup())
			{
				Set(0.1);
			}
			else
			{
				Set(0);
			}
			break;
		case Carry:
			if(IsClawBelowCarry())
			{
				Set(-0.2);
			}
			else
			{
				Set(0);
			}
			break;
		case LaunchHold:
			if(IsClawBelowLaunchHold())
			{
				Set(-0.3);
			}
			else
			{
				Set(0);
			}
			break;
		}
	}
	
	void InitializeVariablesToDefault()
	{
		 PickupPosition= LOW_POT;
		 CarryPosition = HIGH_POT;
		 LaunchHoldPosition = MID_POT;	}
	
	void InitializeVariablesFromParams(FILE* fp)
	{
		 PickupPosition= LOW_POT;
		 CarryPosition = HIGH_POT;
		 LaunchHoldPosition = MID_POT;

		//		fscanf(fp, "%f %f %f", PickupPosition, CarryPosition, LaunchHoldPosition);
	}
	
	void SaveVariablesIntoParams(FILE* fp)
	{
//		fprintf(fp, "%f %f %f", PickupPosition, CarryPosition, LaunchHoldPosition);
	}
	
	void PrintVariablesToSmartDashboard()
	{
//		SmartDashboard::PutNumber("ValueIGotPickupPosition", PickupPosition);
//		SmartDashboard::PutNumber("ValueIGotCarryPosition", CarryPosition);		
//		SmartDashboard::PutNumber("ValueIGotLaunchHoldPosition", LaunchHoldPosition);
	}
	
	void SavePickup(float newValue)
	{
		PickupPosition = newValue;
	}
	
	void SaveCarry(float newValue)
	{
		CarryPosition = newValue;
	}
	
	void SaveLaunchHold(float newValue)
	{
		LaunchHoldPosition = newValue;
	}

	//poteniometer is larger when the claw is down, decreases as we pull tension
	bool IsClawAbovePickup()
	{
		return Potent.GetVoltage() > PickupPosition;
	}
	
	bool IsClawBelowCarry()
	{
		return Potent.GetVoltage() < CarryPosition;
	}
	
	bool IsClawBelowLaunchHold()
	{
		return Potent.GetVoltage() < LaunchHoldPosition;
	}

private:
	//positive speed is release tension
	//negative speed is pull tension
	void Set (float speed)
	{
//		SmartDashboard::PutNumber("CatapultSpeed", speed);
		Left1.Set(speed);
		Left2.Set(speed);
		Right1.Set(-speed);
		Right2.Set(-speed);
	}
		
	CANJaguar & Left1;
	CANJaguar & Left2;
	CANJaguar & Right1;
	CANJaguar & Right2;
	AnalogChannel & Potent;
	bool isLaunchOverride;
	CatapultModes Mode;
	
	float PickupPosition;
	float CarryPosition;
	float LaunchHoldPosition;
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
	Solenoid RollerDown;
	Solenoid RollerUp;
	Solenoid CatapultEnable;
	Solenoid ShiftUp;
	Solenoid ShiftDown;
	
	Compressor Comp;

	AnalogChannel ThrowingPotent;

	DigitalInput LeftRollerUp;
	DigitalInput LeftRollerDown;
	DigitalInput RightRollerUp;
	DigitalInput RightRollerDown;
	
	RobotDrive myRobot; // robot drive system
	Catapult Thrower;
	Joystick DriveStickLeft; // only joystick
	Joystick DriveStickRight;
	Joystick LaunchStick;
	
	bool RollerEnabled;
	bool AutoHasLaunched;
		
	float LaunchTime;
	
	Timer AutoTime;
	
	bool PreviousTrigger;
	int ShiftedCounter;
	
public:
	RobotDemo(void):
		LeftDrive (1),
		RightDrive (2),
		CatapultDriveLeft1 (4),
		CatapultDriveLeft2 (5),
		CatapultDriveRight3 (6),
		CatapultDriveRight4 (7),
		RollerDrive (3),
		RollerDown(1),
		RollerUp(2),
		CatapultEnable(7),
		ShiftUp(3),
		ShiftDown(4),
		
		Comp(1,8),
		
		ThrowingPotent(1),
		LeftRollerUp(9),
		LeftRollerDown(10),
		RightRollerUp(7),
		RightRollerDown(8),
		myRobot(LeftDrive, RightDrive),
		Thrower(CatapultDriveLeft1, CatapultDriveLeft2, CatapultDriveRight3, CatapultDriveRight4, ThrowingPotent),
		DriveStickLeft(1),
		DriveStickRight(2),
		LaunchStick(3),
				
		RollerEnabled(false),
		AutoHasLaunched(false),
		
		LaunchTime(.3),
		PreviousTrigger(true),
		ShiftedCounter(0)

	{		
		LiveWindow::GetInstance()->AddSensor("Thrower","Potentiometer",&ThrowingPotent);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Left1", &CatapultDriveLeft1);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Left2", &CatapultDriveLeft2);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Right1", &CatapultDriveRight3);
		LiveWindow::GetInstance()->AddActuator("Thrower", "Right2", &CatapultDriveRight4);
		
		LiveWindow::GetInstance()->AddSensor("Roller", "LeftUp", &LeftRollerUp);
		LiveWindow::GetInstance()->AddSensor("Roller", "LeftDown", &LeftRollerDown);
		LiveWindow::GetInstance()->AddSensor("Roller", "RightUp", &RightRollerUp);
		LiveWindow::GetInstance()->AddSensor("Roller", "RightDown", &RightRollerDown);
		InitializeVariablesFromParams();
		myRobot.SetExpiration(0.1);
	}

	void InitializeVariablesFromParams()
	{
		FILE* fp = fopen("FRC4646.txt", "r");
		if(fp)
		{
			Thrower.InitializeVariablesFromParams(fp);
			fclose(fp);
		}
		else
		{
			Thrower.InitializeVariablesToDefault();
		}
	}
	
	void SaveVariablesToParams()
	{
		FILE* fp = fopen("FRC4646.txt", "w");
		Thrower.SaveVariablesIntoParams(fp);
		fflush(fp);
		fclose(fp);
	}
	
	void SetPneumaticsSafe()
	{
		RollerUp.Set(true);
		ShiftDown.Set(true);
		CatapultEnable.Set(true);
		PreviousTrigger = false;
		ShiftedCounter = false;
		Wait(0.02);
		RollerUp.Set(false);
		ShiftDown.Set(false);
		CatapultEnable.Set(false);
	}
	
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		ShiftDown.Set(true);
		CatapultEnable.Set(true);
		Wait(0.02);
		ShiftDown.Set(false);
		CatapultEnable.Set(false);
		
		myRobot.Drive(0.5, 0);
		Wait(2);
		myRobot.Drive(0,0);
		
	}
	
	void ProcessDriveStick()
    {
		float driveLeft = DriveStickLeft.GetY();
		float driveRight = DriveStickRight.GetY();
		
		if(DriveStickRight.GetRawButton(2) || DriveStickLeft.GetRawButton(2))
		{
			driveLeft = driveLeft * 0.5;
			driveRight = driveRight * 0.5;
		}
        myRobot.TankDrive(driveRight, driveLeft, true);
		
		bool currentTrigger = DriveStickLeft.GetRawButton(1) || DriveStickRight.GetRawButton(1);
		
		
		if(ShiftedCounter)
		{
			ShiftedCounter --;
			if(ShiftedCounter)
			{
				ShiftDown.Set(false);
				ShiftUp.Set(false);
			}
		}
		
		if(currentTrigger != PreviousTrigger)
		{
			if(currentTrigger)
			{
				ShiftUp.Set(true);
			}
			else
			{
				ShiftDown.Set(true);
			}
			PreviousTrigger = currentTrigger;
			ShiftedCounter = 20;
		}
    }

	float GetAnalogScaled(UINT32 channel, float minimum, float maximum)
	{
		const float rawValue = DriverStation::GetInstance()->GetAnalogIn(channel); //Comes in as 0 to 5
		const float range = maximum - minimum;
		const float scaledValue = (rawValue / 5) * range;
		const float shiftedValue = scaledValue + minimum;
		return shiftedValue;
	}
	
	float ScaleThrottleToPositive(float rawValue)
	{
		const float flippedValue = -rawValue; //-1 to 1
		const float shiftedValue = flippedValue + 1; //0 to 2
		const float scaledValue = shiftedValue *.125; //0 to .25
		return scaledValue;		
	}
	
	void ProcessLaunchStickExtreme3d()
	{
		//ThrowerControl		
		if(LaunchStick.GetRawButton(3))
		{
			Thrower.SetMode(Catapult::Pickup);
		}
		if(LaunchStick.GetRawButton(5))
		{
			Thrower.SetMode(Catapult::Carry);
		}
		if(LaunchStick.GetRawButton(2))
		{
			Thrower.SetMode(Catapult::LaunchHold);
		}
		
		if(LaunchStick.GetRawButton(10))
		{
			Thrower.SetMode(Catapult::Idle);
		}
		
		float realLaunchSpeed = 0.75 + ScaleThrottleToPositive(LaunchStick.GetRawAxis(4));

		static int SendCount = 10;
		SendCount--;
		if(SendCount == 0)
		{
			SendCount = 10;
			SmartDashboard::PutNumber("LaunchSpeed", realLaunchSpeed);
		}
		
		if(LaunchStick.GetRawButton(7))
		{
			realLaunchSpeed = 1;
		}
		else if(LaunchStick.GetRawButton(9))
		{
			realLaunchSpeed = 0.875;
		}
		else if(LaunchStick.GetRawButton(11))
		{
			realLaunchSpeed = 0.75;
		}
		
		if(LaunchStick.GetRawButton(1) || LaunchStick.GetRawButton(7) || LaunchStick.GetRawButton(9) || LaunchStick.GetRawButton(11))
		{
			Thrower.SetMode(Catapult::Idle);
			Comp.Stop();
			Thrower.Launch(realLaunchSpeed, LaunchTime);
			Comp.Start();
		}
		
		if(LaunchStick.GetRawButton(8))
		{
			Thrower.SetMode(Catapult::Manual);
			Thrower.SetManual(-0.2);
		}
		else if(LaunchStick.GetRawButton(12))
		{
			Thrower.SetMode(Catapult::Manual);
			Thrower.SetManual(0.1);
		} else
		{
			Thrower.SetManual(0);
		}
		
		Thrower.ProcessMode();
		
		//RollerControl
		RollerUp.Set(LaunchStick.GetRawButton(6));
		RollerDown.Set(LaunchStick.GetRawButton(4));
		
		float rollerSpeed = LaunchStick.GetY();
		if(rollerSpeed < 0.1 && rollerSpeed > -0.1)
		{
			raollerSpeed = 0;
		}
		RollerDrive.Set(rollerSpeed);		
	}
	
    void OperatorControl(void)
    {
    	Comp.Stop();
        InitializeVariablesFromParams();
        
        NetworkTable * server = NetworkTable::GetTable("SmartDashboard");
		
        myRobot.SetSafetyEnabled(true);
        while(IsOperatorControl() && IsEnabled())
        {
            //ProcessDriveStick();
            //ProcessLaunchStickExtreme3d();
        	
        	''
       	if(DriveStickLeft.GetRawButton(1))
        	{
        		//follow green noodle
        		SmartDashboard::PutNumber("Blobs", server->GetNumber("BLOB_COUNT", 0.0));
        		float blobCount=server->GetNumber("BLOB_COUNT");
        		float greenX=server->GetNumber("GREEN_X");
        		float imageWidth = server->GetNumber("IMAGE_WIDTH", 640);
        		float error = greenX-(imageWidth/2);
        		float scaledError = (error/(imageWidth/2));
        		SmartDashboard::PutNumber("ScaledError",scaledError);
        		SmartDashboard::PutNumber("imageWidth", imageWidth);
        		SmartDashboard::PutNumber("error", error);
        		myRobot.ArcadeDrive(0,-scaledError);
        		
        		
        		
        	}
       	else
       	{
       		myRobot.ArcadeDrive(0.0,0.0);
       	}
            
			Wait(0.005);
		}
		//Comp.Stop();
	}
	
	void Test(void)
	{
		SetPneumaticsSafe();
		Comp.Start();
		bool valuesChanged = false;
		while (IsTest() && IsEnabled())
		{
			LiveWindow::GetInstance()->Run();

			if(LaunchStick.GetRawButton(3))
			{
				Thrower.SavePickup(ThrowingPotent.GetVoltage());
				valuesChanged = true;
			}
			if(LaunchStick.GetRawButton(5))
			{
				Thrower.SaveCarry(ThrowingPotent.GetVoltage());
				valuesChanged = true;
			}
			if(LaunchStick.GetRawButton(2))
			{
				Thrower.SaveLaunchHold(ThrowingPotent.GetVoltage());
				valuesChanged = true;
			}
			
			
			if(LaunchStick.GetRawButton(1))
			{
				if(valuesChanged)
				{
					SaveVariablesToParams();
					valuesChanged = false;
				}
			}
			Wait(0.01);
		}
		Comp.Stop();
	}
	
};

START_ROBOT_CLASS(RobotDemo);
