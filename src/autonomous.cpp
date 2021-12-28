#include "main.h"
#include "globals.h"
#include "macros.h"

void grabMogo() {
    forklift1.goMiddle();
    forklift2.goDown();

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);

    driveTrainPID.moveToPoint({ 0, 24 });

    forklift2.goMiddle();

    pros::delay(250);

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);

    forklift2.goUp();

    driveTrainPID.moveToPoint(Vector2( 0, 10 ), true);

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);

    /*
    startIntakeSmoothMove(false, false);
    intake.counterClockwise(80);

    pros::delay(5000);

    driveTrainPID.move({}, 0);
    intake.stop();
    */
}

void testForklifts() {
    forklift2.goDown();

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);


    forklift2.goUp();

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);
}

void myAuton() {
    trackingData.update({0, 10}, 0);
    // trackingData.update({0, 20}, 0);
    chassis->setState({0_in, 0_in, 90_deg});

    forklift1.enable();
    forklift2.enable();
    intake.enable();

    // testForklifts();
    grabMogo();
}