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

    // Basic op control using tank drive
    while (true) {
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 0);

        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        int intakeMacroCC = masterController.get_digital(DIGITAL_UP);
        int intakeMacroCCW = masterController.get_digital(DIGITAL_DOWN);

        // masterController.clear();
        if (intakeMacroCC == 0 && intakeMacroCCW == 0) {
            stopIntakeSmoothMove();

            // Operator control
            intake.control();
            // printf("Operator control -> Intake");

            if (intakeUp) {
                intake.setPower(60);
            } else if (intakeDown) {
                intake.setPower(-60);
            } else {
                intake.setPower(0);
            }
        } else if (intakeMacroCCW == 1) {
            // Go backwards
            startIntakeSmoothMove(true, true);
        } else if (intakeMacroCC == 1) {
            // Go frowards
            startIntakeSmoothMove(false, false);
        }

        pros::delay(10);
    }    
}