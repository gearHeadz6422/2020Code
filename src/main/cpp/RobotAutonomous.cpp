#include "Robot.h"
#include <cmath>
using namespace std;
#define LEFT 

void Robot::AutonomousInit() { Robot::TeleopInit(); }

void Robot::AutonomousPeriodic() { Robot::TeleopPeriodic(); }