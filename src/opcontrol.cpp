#include "main.h"
#include "globals.h"
#include "macros.h"

/**
 * \brief Scale joystick output to cubic graph
 * 
 * Scales raw value in range [-127, 127] to range [-1, 1] and apply exponent 3 in order to allow for more driver accuracy
*/
double joystickCubicDrive(int raw) {
    double scaledRaw = ((double)raw) / 127.0;
    return pow(scaledRaw, 3) * 127;
}

void myOpControl() {
    // Enable all systems
    intake.enable();
    forklift1.enable();
    forklift2.enable();

    // 0 -> nothing, 1 -> clockwise, -1 -> counter clockwise
    int macroToggle = 0;

    while (true) {
        // Tank drive
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 0);

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

                // Joystick now mapped to intake, change later
                // intake.setPower(joystickCubicDrive(right));
                break;
            }
            default: {
                break;
            }
        }

        if (forklift1Input == 1) {
            // Set to alternate position
            if (forklift1.getState() == Forklift::UP_STATE) {
                forklift1.goDown();
            } else if (forklift1.getState() == Forklift::DOWN_STATE) {
                forklift1.goUp();
            }
        }

        if (forklift2Input == 1) {
            // Set to alternate position
            if (forklift2.getState() == Forklift::UP_STATE) {
                forklift2.goDown();
            } else if (forklift2.getState() == Forklift::DOWN_STATE) {
                forklift2.goUp();
            }
        }

        

        pros::delay(10);
    }    
}