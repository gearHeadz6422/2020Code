#include "Robot.h"

class Gamepad : GenericHID
{

public:
    // These are garbage functions that we don't use, but need to implement from the GenericHID class because they are pure virtual
    double GetX(frc::GenericHID::JoystickHand hand = frc::GenericHID::JoystickHand::kRightHand) const { return -1.0; }
    double GetY(frc::GenericHID::JoystickHand hand = frc::GenericHID::JoystickHand::kRightHand) const { return -1.0; }

    void update()
    {
        for (int i = 0; i < (int)sizeof(buttons); i++)
        {
            buttons[i] = GetRawButton(i + 1);
        }

        leftTrig = GetRawAxis(2);
        rightTrig = GetRawAxis(3);

        leftX = GetRawAxis(0);
        leftY = GetRawAxis(1);
        rightX = GetRawAxis(4);
        rightY = GetRawAxis(5);

        switch (GetPOV())
        {
        case -1:
            dpadPos = "center";
            break;
        case 0:
            dpadPos = "up";
            break;
        case 45:
            dpadPos = "up right";
            break;
        case 90:
            dpadPos = "right";
            break;
        case 135:
            dpadPos = "down right";
            break;
        case 180:
            dpadPos = "down";
            break;
        case 225:
            dpadPos = "down left";
            break;
        case 270:
            dpadPos = "left";
            break;
        case 315:
            dpadPos = "up left";
            break;
        default:
            dpadPos = "center";
            break;
        }
    }

    void postUpdate()
    {
        for (int i = 0; i < (int)sizeof(buttons); i++)
        {
            prevButtons[i] = buttons[i];
        }
    }

    void rumble()
    {
        SetRumble(kLeftRumble, 1.0);
        SetRumble(kRightRumble, 1.0);
    }

    bool getA()
    {
        return buttons[0];
    }
    bool getB()
    {
        return buttons[1];
    }
    bool getX()
    {
        return buttons[2];
    }
    bool getY()
    {
        return buttons[3];
    }
    bool getLB()
    {
        return buttons[4];
    }
    bool getRB()
    {
        return buttons[5];
    }
    bool getStart()
    {
        return buttons[6];
    }
    bool getSelect()
    {
        return buttons[7];
    }
    bool getR3()
    {
        return buttons[8];
    }
    bool getL3()
    {
        return buttons[9];
    }

    double getLeftTrigger()
    {
        return leftTrig;
    }
    double getRightTrigger()
    {
        return rightTrig;
    }

    double getLeftX()
    {
        return leftX;
    }
    double getLeftY()
    {
        return leftY;
    }
    double getRightX()
    {
        return rightX;
    }
    double getRightY()
    {
        return rightY;
    }

    std::string getDpad()
    {
        return dpadPos;
    }

    Gamepad(int desId) : GenericHID(desId)
    {
        dpadPos = "center";

        for (int i = 0; i < (int)sizeof(buttons); i++)
        {
            buttons[i] = false;
            prevButtons[i] = false;
        }
    }

private:
    std::string dpadPos;
    bool buttons[16];
    bool prevButtons[16];

    double leftTrig;
    double rightTrig;

    double leftX;
    double leftY;
    double rightX;
    double rightY;
};