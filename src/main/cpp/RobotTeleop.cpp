#include "Robot.h"
#include <cmath>

using namespace std;

bool motorDebug = false;

void Robot::TeleopInit()
{
	// Set the initial state for our variables, and pull some data from the driver station
	accelX = 0.0;
	accelY = 0.0;
	lastAccelX = 0.0;
	lastAccelY = 0.0;
	liftEncoderHigh.Reset();
	leftEncoder.Reset();
	rightEncoder.Reset();
	grabberEncoder.Reset();

	if (sensorBoardType == "navx")
	{
		navx->ZeroYaw();
	}
	else
	{
		// analogDev.Reset();
	}

	currentAnlge = 0;
	prevAnlge = 0;

	// Sets the way that the Talons will recieve input (We think. In practice changing the mode mysteriously does nothing)
	FrontLeft.Set(ControlMode::PercentOutput, 0);
	FrontRight.Set(ControlMode::PercentOutput, 0);
	BackLeft.Set(ControlMode::PercentOutput, 0);
	BackRight.Set(ControlMode::PercentOutput, 0);

	// This is test code for master/slave function - DOESN'T WORK
	// testTalon2.Set(testTalon1.GetDeviceID());
	// testTalon2.Set(ControlMode::Follower, 2);
	testTalon1.Set(ControlMode::PercentOutput, 0);
	testTalon2.Set(ControlMode::PercentOutput, 0);
	testTalon3.Set(ControlMode::PercentOutput, 0);
	testTalon4.Set(ControlMode::PercentOutput, 0);
	testTalon5.Set(ControlMode::PercentOutput, 0);
	testTalon6.Set(ControlMode::PercentOutput, 0);
	testTalon7.Set(ControlMode::PercentOutput, 0);
	testTalon8.Set(ControlMode::PercentOutput, 0);
	testTalon9.Set(ControlMode::PercentOutput, 0);

	// Initializes the encoder attatched to the lower lift's talon
	// liftLow.SetSelectedSensorPosition(0, 0, 0);

	// Tracks the current step the auto-lineup algorithim is on
	targetAngle = -1;

	// This sets up the ultrasonic sensors and the box's string potentiometer, and declares that they are hooked into analog port 0
	frontUltraSonic = new AnalogInput(0);
	rearUltraSonic = new AnalogInput(1);
	boxPotentiometer = new AnalogInput(2);

	// Turns on the compressor and sets the solenoid for the grabber to off
	compressor->SetClosedLoopControl(true);
	grabber.Set(frc::DoubleSolenoid::kReverse);

	if (motorDebug)
	{
		// Create the test motor entry on the shuffle board
		frc::SmartDashboard::PutNumber("testMotor", -1);
	}

	// Initialize the hatch indicator to off
	frc::SmartDashboard::PutBoolean("Holding hatch", false);

	// Create an entry on the dashboard that allows us to tune the speed of the robot at presentations and stuff like that
	frc::SmartDashboard::PutNumber("Child endangerment multiplier", -1);
}

void Robot::TeleopPeriodic()
{
	driverController.rumble();
	// The Gamepad class polls the controller only once per frame for performance reasons, using the update function
	driverController.update();
	copilotController.update();

	// This is the code that will cancel operations that are posing real world danger. This stuff is very important, so it comes first, and isn't dependent on any other systems actually working. If you need to change it for some reason, make sure it still works!
	bool stickMoved = false; // This is used later to cancel out of autonomous movements when the driver moves the stick
	if (driverController.getStart() || driverController.getStart())
	{
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);

		testTalon1.Set(0.0);
		testTalon2.Set(0.0);
		testTalon3.Set(0.0);
		testTalon4.Set(0.0);
		testTalon5.Set(0.0);
		testTalon6.Set(0.0);
		testTalon7.Set(0.0);
		testTalon8.Set(0.0);
		testTalon9.Set(0.0);

		return;
	}

	// This is used for debugging specific motors and TALONs
	if (motorDebug)
	{
		testMotor = frc::SmartDashboard::GetNumber("testMotor", -1);

		switch (testMotor)
		{
		case 1:
			testTalon1.Set(driverController.getLeftY());
			break;
		case 2:
			testTalon2.Set(driverController.getLeftY());
			break;
		case 3:
			testTalon3.Set(driverController.getLeftY());
			break;
		case 4:
			testTalon4.Set(driverController.getLeftY());
			break;
		case 5:
			testTalon5.Set(driverController.getLeftY());
			break;
		case 6:
			testTalon6.Set(driverController.getLeftY());
			break;
		case 7:
			testTalon7.Set(driverController.getLeftY());
			break;
		case 8:
			testTalon8.Set(driverController.getLeftY());
			break;
		case 9:
			testTalon9.Set(driverController.getLeftY());
			break;
		}
		return;
	}
	else
	{
		testTalon1.Set(0.0);
		testTalon2.Set(0.0);
		testTalon3.Set(0.0);
		testTalon4.Set(0.0);
		testTalon5.Set(0.0);
		testTalon6.Set(0.0);
		testTalon7.Set(0.0);
		testTalon8.Set(0.0);
		testTalon9.Set(0.0);
	}

	// Now, we collect all of the data our sensors are spitting out, process it, and display some of it on the smart dashboard
	if (sensorBoardType == "navx")
	{
		accelX = navx->GetWorldLinearAccelX() * 1000;
		accelY = navx->GetWorldLinearAccelY() * 1000;
	}
	else
	{
		// accelX = analogDev.GetAccelX() * 1000;
		// accelY = analogDev.GetAccelY() * 1000;
	}

	frontWallDistance = frontUltraSonic->GetValue();
	rearWallDistance = rearUltraSonic->GetValue();
	frc::SmartDashboard::PutNumber("Front wall distance", frontWallDistance);
	frc::SmartDashboard::PutNumber("Rear wall distance", rearWallDistance);

	if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500)
	{ // TODO: Test and make sure this still works
		colliding = true;
	}
	else
	{
		colliding = false;
	}

	frc::Color detectedColor = colorSensor.GetColor();
	frc::SmartDashboard::PutNumber("Color-Red", detectedColor.red);
	frc::SmartDashboard::PutNumber("Color-Green", detectedColor.green);
	frc::SmartDashboard::PutNumber("Color-Blue", detectedColor.blue);

	// This returns value in terms of bits so we convert it to rotations
	// liftHeightHigh = liftEncoderHigh.GetDistance() / -1024;
	// liftHeightLow = liftLow.GetSelectedSensorPosition(0) / 4096.0;

	leftDistance = leftEncoder.GetDistance() / 360;
	rightDistance = rightEncoder.GetDistance() / 360;

	frc::SmartDashboard::PutNumber("Left encoder", leftDistance);
	frc::SmartDashboard::PutNumber("Right encoder", rightDistance);

	frontCameraOutput = frc::SmartDashboard::GetNumber("Front camera out", 0);
	rearCameraOutput = frc::SmartDashboard::GetNumber("Rear camera out", 0);

	if (sensorBoardType == "navx")
	{
		currentAnlge += navx->GetAngle() - prevAnlge;
	}
	else if (sensorBoardType == "ADI")
	{
		// currentAnlge += analogDev.GetAngle() - prevAnlge;
	}

	frc::SmartDashboard::PutNumber("Grabber encoder", grabberEncoder.GetDistance());

	// We need to make sure that the robot always has an idea of the direction it's pointing, so we manipulate the angle measurement a little so that positive angles are always right, negative angles are always left, and that the angle is never above 180
	while (currentAnlge >= 360)
	{
		currentAnlge -= 360;
	}
	while (currentAnlge <= -360)
	{
		currentAnlge += 360;
	}
	if (currentAnlge < -180 || currentAnlge > 180)
	{
		currentAnlge = -currentAnlge;
	}
	frc::SmartDashboard::PutNumber("Angle", currentAnlge);

	float big = 0.0; // This is used by the mecanum drive code

	// Calculates the robot's speed multiplier
	float multiplier = driverController.getLeftTrigger() + 1;
	if (frc::SmartDashboard::GetNumber("Child endangerment multiplier", -1) != -1)
	{
		multiplier = frc::SmartDashboard::GetNumber("Child endangerment multiplier", 1);
	}

	if (mecanumDrive)
	{
		if (fabs(driverController.getRightX()) > fabs(driverController.getRightY()) && fabs(driverController.getRightX()) > fabs(driverController.getLeftX()))
		{
			big = fabs(driverController.getRightX());
		}
		else if (fabs(driverController.getRightY()) > fabs(driverController.getRightX()) && fabs(driverController.getRightY()) > fabs(driverController.getLeftX()))
		{
			big = fabs(driverController.getRightY());
		}
		else
		{
			big = fabs(driverController.getLeftX());
		}

		FrontLeft.Set(((cos(atan2(driverController.getRightY(), driverController.getRightX()) - 0.7853981633974483) + driverController.getLeftX() * (1 - driverController.getRightTrigger())) / 2) * big * multiplier);
		FrontRight.Set(((sin(atan2(driverController.getRightY(), driverController.getRightX()) - 0.7853981633974483) - driverController.getLeftX() * (1 - driverController.getRightTrigger())) / 2) * big * multiplier);
		BackLeft.Set(((sin(atan2(driverController.getRightY(), driverController.getRightX()) - 0.7853981633974483) + driverController.getLeftX() * (1 - driverController.getRightTrigger())) / 2) * big * multiplier);
		BackRight.Set(((cos(atan2(driverController.getRightY(), driverController.getRightX()) - 0.7853981633974483) - driverController.getLeftX() * (1 - driverController.getRightTrigger())) / 2) * big * multiplier);
	}
	else
	{ // Tank drive
		FrontLeft.Set(driverController.getLeftY());
		BackLeft.Set(driverController.getLeftY());
		FrontRight.Set(driverController.getRightY());
		BackRight.Set(driverController.getRightY());
	}

	// End driver code; Begin co-pilot code -------------------------------------------------------------------------------------------------------------------

	// EXAMPLE CODE
	// if (xButton2)
	// {
	// 	grabber.Set(frc::DoubleSolenoid::kForward);
	// }
	// else if (bButton2)
	// {
	// 	grabber.Set(frc::DoubleSolenoid::kReverse);
	// }

	// End controller code ------------------------------------------------------------------------------------------------------------------------------------

	// Put your debugging code here ===============

	// ============================================

	// Finally update our variables that track data from previous robot ticks
	driverController.postUpdate();
	copilotController.postUpdate();

	lastAccelX = accelX;
	lastAccelY = accelY;

	if (sensorBoardType == "navx")
	{
		prevAnlge = navx->GetAngle();
	}
	else
	{
		// prevAnlge = analogDev.GetAngle();
	}
}
