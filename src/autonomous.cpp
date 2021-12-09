#include "main.h"
#include "globals.h"

void myAuton() {
    display.setMode(PID_GRAPH);
    driveTrainPID.rotateTo(degToRad(90)); // Rotate to 90 degrees   
}