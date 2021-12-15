#include "main.h"
#include "globals.h"

void myAuton() {
    driveTrainPID.rotateTo(degToRad(90)); // Rotate to 90 degrees
    pros::delay(200);
    driveTrainPID.moveToPoint(Vector2( 1, 0 ));
}