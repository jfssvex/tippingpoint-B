#include "main.h"
#include "globals.h"

void myAuton() {
    tLeft.set_brake_mode(MOTOR_BRAKE_BRAKE);
    tRight.set_brake_mode(MOTOR_BRAKE_BRAKE);
    bLeft.set_brake_mode(MOTOR_BRAKE_BRAKE);
    bRight.set_brake_mode(MOTOR_BRAKE_BRAKE);

    driveTrainPID.moveToPoint(Vector2( 25, 20 ));
    // pros::delay(250);
    driveTrainPID.rotateTo(degToRad(0)); // Rotate to 90 degrees
    // driveTrainPID.moveToPoint(Vector2( 0, 0 ));

    

   /*
    printTracking = true;

    driveTrain->forward(64);
    pros::delay(20000);
    driveTrain->forward(0);

    pros::delay(500);

    printTracking = false;
    */
}