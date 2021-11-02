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
    // Basic op control using tank drive
    while (true) {
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);

        if (left > 60) {
            printf("hello wrld!!!!");
        }

        
        // TODO: Change threshold to something useful
        driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), 10);

        pros::delay(5);
    }    
}