#include "main.h"
#include "globals.h"

void myAuton() {
    // trackingData.update({0, 20}, 0);
    chassis->setState({0_in, 0_in, 90_deg});

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

    // forklift1.goUp();
    // forklift2.goUp();

    forklift1.goMiddle();
    forklift2.goDown();

    chassis->driveToPoint({ 0_ft, 1_ft });
    // driveTrainPID.experimentalMoveToPoint({ 0, 12 });
    // chassis->moveDistance(1.5_ft);

    // Wait for it to reach okay tolerance levels
    do {
        // printf("Error: %f\n", forklift2.getError());
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);

    printf("Forklift 2 error: %f\n", abs(forklift2.getError()));

    forklift2.goMiddle();

    // Wait for it to reach okay tolerance levels
    do {
        // printf("Error: %f\n", forklift2.getError());
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);

    chassis->driveToPoint({ 0_ft, 0_ft }, true);

    forklift2.goDown();
    

   /*
    printTracking = true;

    driveTrain->forward(64);
    pros::delay(20000);
    driveTrain->forward(0);

    pros::delay(500);

    printTracking = false;
    */
}