/*
 * RobotUtils.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */
#include "Robot.h"

using namespace std;

void Robot::InitEncoder(frc::Encoder &enc) {
	/* Defines the number of samples to average when determining the rate.
	 * On a quadrature encoder, values range from 1-255; larger values
	 *   result in smoother but potentially less accurate rates than lower
	 *   values.
	 */
	//enc.SetSamplesToAverage(5);

	/* Defines how far the mechanism attached to the encoder moves per pulse.
	 * In this case, we assume that a 360 count encoder is directly attached
	 * to a 3 inch diameter (1.5inch radius) wheel, and that we want to
	 * measure distance in inches.
	 */
	//enc.SetDistancePerPulse(1.0 / 180.0 * M_PI * 2.5);

	/* Defines the lowest rate at which the encoder will not be considered
	 *   stopped, for the purposes of the GetStopped() method.
	 * Units are in distance / second, where distance refers to the units
	 *   of distance that you are using, in this case inches.
	 */
	//enc.SetMinRate(1.0);
}

void
Robot::MotorSpeedLeft(const float val)
{
	FrontLeft.Set(val);
	BackLeft.Set(val);
}

void
Robot::MotorSpeedRight(const float val)
{
	FrontRight.Set(val);
	BackRight.Set(val);
}


// the light is connected to port 0 of the Pneumatic Control Module
void
Robot::CameraLightOn()
{
	//m_solenoid.Set(1);
	//m_solenoid_gear.Set(1);
	return;
}
void
Robot::CameraLightOff()
{
	//m_solenoid.Set(0);
	//m_solenoid_gear.Set(0);
	return;
}