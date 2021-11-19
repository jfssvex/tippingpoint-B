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
    return pow(scaledRaw, 3) * 127;
}

void myOpControl() {
    // Enable all systems
    intake.enable();
    forklift1.enable();
    forklift1.fullReset();

    // 0 -> nothing, 1 -> clockwise, -1 -> counter clockwise
    int macroToggle = 0;

    // Internal state for forklift 1
    // 0 -> Down, 1 -> Middle, 2 -> Up
    int forklift1State = 0;

    while (true) {
        // Tank drive
        int forward = masterController.get_analog(ANALOG_LEFT_Y);
        int yaw = masterController.get_analog(ANALOG_RIGHT_X);
        driveTrain->arcade(forward, yaw, 0);

        // Intake manual controls
        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        // Forklift manual controls
        int forklift1Input = masterController.get_digital_new_press(DIGITAL_L2);
        int forklift2Input = masterController.get_digital_new_press(DIGITAL_R2);

        // Intake macro
        int intakeMacroCW = masterController.get_digital_new_press(DIGITAL_UP);
        int intakeMacroCCW = masterController.get_digital_new_press(DIGITAL_DOWN);

        // Intake macro handler
        if (macroToggle == -1 && intakeMacroCCW) {
            // Turn off macro
            macroToggle = 0;
        } else if (macroToggle == 1 && intakeMacroCW) {
            // Turn off macro
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

                int speed = 60; // Also can be 40

                if (intakeUp) {
                    intake.setPower(speed);
                } else if (intakeDown) {
                    intake.setPower(-speed);
                } else {
                    intake.setPower(0);
                }
                break;
            }
            default: {
                break;
            }
        }

        if (forklift1Input == 1) {
            forklift1State += 1;
            forklift1State %= 3; // There are only 3 states available, modulo to 3
            
            // Set to alternate position
            if (forklift1State == 0) {
                // Go down
                forklift1.goDown();
                display.logMessage("Forklift 1: going down.");
            } else if (forklift1State == 1) {
                // Go middle
                forklift1.goMiddle();
                display.logMessage("Forklift 1: going middle.");
            } else if (forklift1State == 2) {
                // Go up
                forklift1.goUp();
                display.logMessage("Forklift 1: going up.");
            }
        }

        if (forklift2Input == 1) {
            display.logMessage("Forklift 2: clicked");
            /*
            // Set to alternate position
            if (forklift2.getState() == Forklift::UP_STATE) {
                forklift2.goDown();
            } else if (forklift2.getState() == Forklift::DOWN_STATE) {
                forklift2.goUp();
            }
            */
        }

        

        pros::delay(10);
    }    
}