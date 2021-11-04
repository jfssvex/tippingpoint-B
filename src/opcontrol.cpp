#include "main.h"
#include "globals.h"

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

    // Basic op control using tank drive
    while (true) {
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 0);

        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        // masterController.clear();
        
        if (intakeUp) {
            intakeMotor.move(127);
            masterController.set_text(0, 0, "Up  ");
        } else if (intakeDown) {
            intakeMotor.move(-127);
            masterController.set_text(0, 0, "Down");
        } else {
            intakeMotor.move(0);
            masterController.set_text(0, 0, "None");
        }

        pros::delay(10);
    }    
}