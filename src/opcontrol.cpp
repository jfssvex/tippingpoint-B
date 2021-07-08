#include "main.h"
#include "globals.h"

void myOpControl() {
    // Basic op control using tank drive
    while (true) {
        int power = masterController.get_analog(ANALOG_LEFT_Y);
        int turn = masterController.get_analog(ANALOG_RIGHT_X);

        driveTrain->arcade(power, turn, 0.5); // TODO: Change threshold to something useful
    }    
}