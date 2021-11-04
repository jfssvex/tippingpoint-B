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
    display.logMessage("Running OPControl!!!");

    int intakeMacroState = 0; // 0 -> Not on, 1 -> Clockwise, -1 -> Counter-Clockwise

    // Basic op control using tank drive
    while (true) {
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 0);

        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        int intakeMacroCC = masterController.get_digital_new_press(DIGITAL_UP);
        int intakeMacroCCW = masterController.get_digital_new_press(DIGITAL_DOWN);

        // Toggle macro based on button
        if (intakeMacroState == 1 && intakeMacroCC == 1) {
            intakeMacroState = 0;
        } else if (intakeMacroState == -1 && intakeMacroCCW == 1) {
            intakeMacroState = 0;
        } else {
            intakeMacroState = intakeMacroCC + (-1 * intakeMacroCCW);
        }

        // masterController.clear();
        if (intakeMacroState == 0) {
            // Operator control
            intake.control();

            if (intakeUp) {
                intake.setPower(42);
            } else if (intakeDown) {
                intake.setPower(42);
            } else {
                intake.setPower(0);
            }
        } else if (intakeMacroState == -1) {
            // Go backwards
            startIntakeSmoothMove(true, true);
        } else if (intakeMacroState == 1) {
            // Go frowards
            startIntakeSmoothMove(false, false);
        }

        pros::delay(10);
    }    
}