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
    // Enable the intake
    intake.enable();

    // 0 -> nothing, 1 -> clockwise, -1 -> counter clockwise
    int macroToggle = 0;

    while (true) {
        // Basic op control using tank drive
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 0);

        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        /*
        // For experimenting with speeds
        int intakeSpeed2 = masterController.get_digital(DIGITAL_L2);
        int intakeSpeed3 = masterController.get_digital(DIGITAL_R2);
        */
       
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

                /*
                int speed = 40;

                if (intakeSpeed3) {
                    speed = 100;
                } else if (intakeSpeed2) {
                    speed = 60;
                }

                if (intakeUp) {
                    intake.setPower(speed);
                } else if (intakeDown) {
                    intake.setPower(-speed);
                } else {
                    intake.setPower(0);
                }
                */

                // Joystick now mapped to intake, change later
                intake.setPower(joystickCubicDrive(right));
                break;
            }
            default: {
                break;
            }
        }

        // Operator control
        if (!macroToggle) {
            stopIntakeSmoothMove();
        }

        pros::delay(10);
    }    
}