#include "main.h"
#include "globals.h"

void myAuton() {
    // trackingData.update({0, 20}, 0);
    forklift1.enable();
    forklift2.enable();
    intake.enable();

    /*
    forklift1.goDown();
    forklift2.goDown();

    intake.clockwise(80);

    pros::delay(3000);


    forklift1.goMiddle();
    forklift2.goMiddle();
    intake.counterClockwise(80);

    pros::delay(3000);

    forklift1.goUp();
    forklift2.goUp();

    intake.stop();

    pros::delay(3000);
    */

    // forklift2.goUp();

    // forklift1.goMiddle();
    // forklift2.goDown();

    forklift1.goUp();
    forklift2.goUp();

    // driveTrainPID.moveToPoint(Vector2( 0, 15 ));
    // driveTrainPID.moveToPoint(Vector2( 0, 0 ), false);

    // // Wait for it to reach okay tolerance levels
    // do {
    //     // printf("Error: %f\n", forklift2.getError());
    //     pros::delay(10);
    // } while (abs(forklift2.getError()) > 40);

    // forklift2.goUp();

    // // Wait for it to reach okay tolerance levels
    // do {
    //     // printf("Error: %f\n", forklift2.getError());
    //     pros::delay(10);
    // } while (abs(forklift2.getError()) > 40);



    // pros::delay(250);
    // driveTrainPID.rotateTo(degToRad(0)); // Rotate to 90 degrees
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