#include "main.h"
#include "globals.h"
#include "macros.h"

/**
 * \brief Scale joystick output to cubic graph
 * 
 * Scales raw value in range [-127, 127] to range [-1, 1] and 
 * apply exponent 3 in order to allow for more driver accuracy
*/
double joystickCubicDrive(int raw) {
    double scaledRaw = ((double)raw) / 127.0;
    return raw < 0 ? pow(scaledRaw, 2) * -127 : pow(scaledRaw, 2) * 127;
}

void myOpControl() {
    // Enable all systems
    intake.enable();
    forklift1.enable();

    // 0 -> nothing, 1 -> clockwise, -1 -> counter clockwise
    int macroToggle = 0;

    // Internal state for forklift 1
    // 0 -> Down, 1 -> Middle, 2 -> Up
    int forklift1State = 0;

    while (true) {
        // Arcade drive controls
        // int forward = masterController.get_analog(ANALOG_LEFT_Y);
        // int yaw = masterController.get_analog(ANALOG_RIGHT_X);

        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);

        // Brake controls for drifting???
        // int brakeLeft = masterController.get_digital(DIGITAL_L2);
        // int brakeRight = masterController.get_digital(DIGITAL_R2);

        // Intake manual controls
        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        // Forklift 1 manual controls
        int forklift1Up = masterController.get_digital(DIGITAL_UP);
        int forklift1Down = masterController.get_digital(DIGITAL_DOWN);

        // Forklift 2 manual controls
        int forklift2Up = masterController.get_digital(DIGITAL_X);
        int forklift2Down = masterController.get_digital(DIGITAL_B);

        // Intake macro
        int intakeMacroCW = masterController.get_digital_new_press(DIGITAL_LEFT);
        int intakeMacroCCW = masterController.get_digital_new_press(DIGITAL_RIGHT);

        // Pass joystick values to drivetrain
        // driveTrain->arcade(forward, yaw, 0);
        // driveTrain->arcadeWithBrakes(forward, yaw, brakeLeft, brakeRight, 0);
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 0);

        // Intake macro handler
        if (macroToggle != 0 && intakeMacroCCW) {
            // Turn off macro if already on
            macroToggle = 0;
        } else {
            // State does not match the button pressed, enable the respective macro
            if (intakeMacroCW) {
                // Change macro state to clockwise
                macroToggle = 1;
            } else if (intakeMacroCCW) {
                // Change macro state to counter clockwise
                macroToggle = -1;
            }
        }

        // Apply macro control to intake system
        switch (macroToggle) {
            case 1: {
                startIntakeSmoothMove(false, false);
                break;
            }
            case -1: {
                startIntakeSmoothMove(true, true);
                break;
            }
            case 0: {
                // Operator control
                intake.control();

                int intakeSpeed = 80; // Also can be 40

                if (intakeUp) {
                    intake.setPower(intakeSpeed);
                } else if (intakeDown) {
                    intake.setPower(-intakeSpeed);
                } else {
                    intake.setPower(0);
                }
                break;
            }
            default: {
                break;
            }
        }
        intake.update();

        // Forklift 1 control
        forklift1.control();
        int forkliftSpeed = 127;

        if (forklift1Up) {
            forklift1.setPower(forkliftSpeed);
        } else if (forklift1Down) {
            forklift1.setPower(-forkliftSpeed);
        } else {
            forklift1.setPower(0);
        }

        // Run update funcs on sysmans
        forklift1.update();

        pros::delay(10);
    }    
}