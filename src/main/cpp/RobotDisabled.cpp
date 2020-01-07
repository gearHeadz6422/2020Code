/*
 * RobotDisabled.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"

void
Robot::DisabledInit()
{
	//CameraLightOff();
	FrontLeft.Set(0.0);
	FrontRight.Set(0.0);
	BackLeft.Set(0.0);
	BackRight.Set(0.0);
	//maxpwr1 = 0;
	//maxpwr2 = 0;
	waitper = 0;
	winchSpeed = 0;
	//talon_winch.Set(0.0);
	//talon_ballpick.Set(0.0);
	//talon_ballshoot.Set(0.0);
}
