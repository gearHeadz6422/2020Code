#include "Robot.h"
#include <unistd.h>

//static void VisionThread();

using namespace std;

Robot::Robot() :
	m_pdp(0),

	FrontLeft(1),
	BackLeft(2),
	FrontRight(3),
	BackRight(4),
	liftLow(5),
	grabberWinch(6),
	intake(7),
	liftHigh(9),

	testTalon1(1),
	testTalon2(2),
	testTalon3(3),
	testTalon4(4),
	testTalon5(5),
	testTalon6(6),
	testTalon7(7),
	testTalon8(8),
	testTalon9(9)
{

	try {
		navx = new AHRS(SPI::Port::kMXP);
		int n = 0;
		while (n < 20){
			n++;
			if (navx->IsCalibrating()) {
				sleep(1);
			} else {
				break;
			}
		}
	} catch (std::exception& ex) {
		std::string err_string = "Error instantiating navX MXP:  ";
		err_string += ex.what();
		DriverStation::ReportError(err_string.c_str());
	}

	FrontRight.SetInverted(true);
	BackRight.SetInverted(true);

	InitEncoder(liftEncoderHigh);
	angle = navx->GetAngle();
	navx->ZeroYaw();
	memset(maxpwr,0,sizeof(maxpwr));
}

void Robot::RobotInit() {
		// Set up the initial state of our shuffleb	oard
	frc::SmartDashboard::PutNumber("desAutoDelay", 0);
}
int main() { return frc::StartRobot<Robot>(); }