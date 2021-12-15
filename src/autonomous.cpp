#include "main.h"
#include "globals.h"

void myAuton() {
    /*
    driveTrainPID.rotateTo(degToRad(90)); // Rotate to 90 degrees
    pros::delay(200);
    driveTrainPID.moveToPoint(Vector2( 1, 0 ));
    */
   
    printTracking = true;

    // driveTrain->forward(64);
    pros::delay(20000);
    driveTrain->forward(0);

    pros::delay(500);

    printTracking = false;
}