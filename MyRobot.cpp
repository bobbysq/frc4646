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

const float LOW_POT = 1.566;
const float MID_POT = 1.951;
const float HIGH_POT = 3.179;

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
		 LaunchHoldPosition = MID_POT;

		PickupPosition = 2.15;
		CarryPosition = 1.9;
		LaunchHoldPosition = 2;	 	
	}
	
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
//	Solenoid BackboardOut;
//	Solenoid BackboardIn;
	Solenoid RollerDown;
	Solenoid RollerUp;
//	Solenoid DefenceUp;
//	Solenoid DefenceDown;
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
	
public:
	RobotDemo(void):
		LeftDrive (1),
		RightDrive (2),
		CatapultDriveLeft1 (4),
		CatapultDriveLeft2 (5),
		CatapultDriveRight3 (6),
		CatapultDriveRight4 (7),
		RollerDrive (3),
//		BackboardOut(8),
//		BackboardIn(7),
		RollerDown(1),
		RollerUp(2),
//		DefenceUp(6),
//		DefenceDown(5),
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
		
		LaunchTime(.3)

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
//		BackboardIn.Set(true);
		RollerUp.Set(true);
//		DefenceUp.Set(true);
		ShiftDown.Set(true);
		CatapultEnable.Set(true);
		Wait(0.02);
//		BackboardIn.Set(false);
		RollerUp.Set(false);
//		DefenceUp.Set(false);
		ShiftDown.Set(false);
		CatapultEnable.Set(false);
	}
	
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		CatapultEnable.Set(true);
		Wait(0.02);
		CatapultEnable.Set(false);
		
		myRobot.Drive(0.5, 0);
		Wait(2);
		myRobot.Drive(0,0);
		
		//		float timeForRollersToGoUp = DriverStation::GetInstance()->GetAnalogIn(1);
//		float powerForDrive = DriverStation::GetInstance()->GetAnalogIn(2);
//		float timeForDrive = DriverStation::GetInstance()->GetAnalogIn(3);
//		float powerForLaunch = DriverStation::GetInstance()->GetAnalogIn(4);
//		
//		AutoTime.Reset();
//		AutoTime.Start();
//		myRobot.SetSafetyEnabled(false);
//		//SetPneumaticsSafe();
//		ShiftUp.Set(true);
//		RollerDown.Set(true);
//		CatapultEnable.Set(true);
//		Wait(0.02);
//		ShiftUp.Set(false);
//		RollerDown.Set(false);
//		CatapultEnable.Set(false);
//				
//		Thrower.SetMode(Catapult::LaunchHold);
//		RollerDrive.Set(0.3);
//		while(!Thrower.IsClawBelowLaunchHold())
//		{
//			Thrower.ProcessMode();
//			Wait(0.005);
//		}
//		
//		
//		RollerUp.Set(true);
//		Wait(0.02);
//		RollerUp.Set(false);
//		
//		
//		Wait(timeForRollersToGoUp);
//		myRobot.Drive(powerForDrive, 0);
//		Wait(timeForDrive);
//		Thrower.Launch(powerForLaunch, LaunchTime);
//		myRobot.Drive(0, 0);
	}
	
	void ProcessDriveStick()
    {

		float driveLeft = DriveStickLeft.GetY();
		float driveRight = DriveStickRight.GetY();
		
		if(DriveStickRight.GetRawButton(2))
		{
			driveLeft = driveLeft * 0.5;
			driveRight = driveRight * 0.5;
		}
		
		if(DriveStickRight.GetRawButton(1))
		{
			driveLeft = driveRight;
		}
		
        myRobot.TankDrive(driveRight, driveLeft, true);
        
        ShiftUp.Set(DriveStickRight.GetRawAxis(6) < -.5);
        ShiftDown.Set(DriveStickRight.GetRawAxis(6) > .5);
        
//        BackboardIn.Set(DriveStickRight.GetRawButton(7));
//        BackboardOut.Set(DriveStickRight.GetRawButton(8));
//        DefenceUp.Set(DriveStickRight.GetRawButton(9));
//        DefenceDown.Set(DriveStickRight.GetRawButton(10));
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
		const float scaledValue = shiftedValue *.25; //0 to .5
		return scaledValue;		
	}
	
	void ProcessLaunchStickOther()
	{
//		float realLaunchSpeed = GetAnalogScaled(1, .5, 1);
		float realLaunchSpeed = 0.5 + ScaleThrottleToPositive(LaunchStick.GetZ());
//		SmartDashboard::PutNumber("LaunchSpeed", realLaunchSpeed);
		//ThrowerControl		
		if(LaunchStick.GetRawButton(5))
		{
			Thrower.SetMode(Catapult::Pickup);
		}
		if(LaunchStick.GetRawButton(4))
		{
			Thrower.SetMode(Catapult::Carry);
		}
		
		if(LaunchStick.GetRawButton(7) || LaunchStick.GetRawButton(10))
		{
			Thrower.SetMode(Catapult::Idle);
		}
		if(LaunchStick.GetRawButton(1))
		{
			Thrower.SetMode(Catapult::Idle);
			Comp.Stop();
			Thrower.Launch(realLaunchSpeed, LaunchTime);
			Comp.Start();
		}
		
		if(LaunchStick.GetRawButton(11))
		{
			Thrower.SetMode(Catapult::Manual);
			Thrower.SetManual(0.1);
		}
		else if(LaunchStick.GetRawButton(6))
		{
			Thrower.SetMode(Catapult::Manual);
			Thrower.SetManual(-0.1);
		} else
		{
			Thrower.SetManual(0);
		}
		
		Thrower.ProcessMode();
		
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
	
	void ProcessLaunchStickExtreme3d()
	{
		float realLaunchSpeed = 0.5 + ScaleThrottleToPositive(LaunchStick.GetRawAxis(4));
//		SmartDashboard::PutNumber("LaunchSpeed", realLaunchSpeed);
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
		
		if(LaunchStick.GetRawButton(9) || LaunchStick.GetRawButton(10))
		{
			Thrower.SetMode(Catapult::Idle);
		}
		if(LaunchStick.GetRawButton(1))
		{
			Thrower.SetMode(Catapult::Idle);
			Comp.Stop();
			Thrower.Launch(realLaunchSpeed, LaunchTime);
			Comp.Start();
		}
		
		if(LaunchStick.GetRawButton(7) || LaunchStick.GetRawButton(8))
		{
			Thrower.SetMode(Catapult::Manual);
			Thrower.SetManual(-0.2);
		}
		else if(LaunchStick.GetRawButton(11) || LaunchStick.GetRawButton(12))
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
			rollerSpeed = 0;
		}
		RollerDrive.Set(rollerSpeed);		
	}
	
    void OperatorControl(void)
    {
    	Comp.Start();
        InitializeVariablesFromParams();
        
//		SmartDashboard::PutNumber("ValueIGotLaunchTime", LaunchTime);		
//		Thrower.PrintVariablesToSmartDashboard();
		
        myRobot.SetSafetyEnabled(true);
        while(IsOperatorControl() && IsEnabled())
        {
            ProcessDriveStick();
			
//            ProcessLaunchStickOther();
            ProcessLaunchStickExtreme3d();
            
//            SmartDashboard::PutNumber("PotentiometerValue" , ThrowingPotent.GetVoltage());
//			SmartDashboard::PutNumber("LeftRollerUp", LeftRollerUp.Get());
//			SmartDashboard::PutNumber("LeftRollerDown", LeftRollerDown.Get());
//			SmartDashboard::PutNumber("RightRollerUp", RightRollerUp.Get());
//			SmartDashboard::PutNumber("RightRollerDown", RightRollerDown.Get());
			Wait(0.005);
		}
		Comp.Stop();
	}
	
	void Test(void)
	{
		SetPneumaticsSafe();
		Comp.Start();
		bool valuesChanged = false;
		while (IsTest() && IsEnabled())
		{
			LiveWindow::GetInstance()->Run();

//			if(LaunchStick.GetRawButton(5))
			if(LaunchStick.GetRawButton(3))
			{
				Thrower.SavePickup(ThrowingPotent.GetVoltage());
				valuesChanged = true;
			}
//			if(LaunchStick.GetRawButton(4))
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
