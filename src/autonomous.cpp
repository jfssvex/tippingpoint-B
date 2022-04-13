#include "main.h"
#include "globals.h"
#include "macros.h"

void grabMogo() {
    forklift1.goDown();
    forklift2.goUp();

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift1.getError()) > 40);

    driveTrainPID.moveToPoint({ 0, 36 });

    forklift1.goUp();
    forklift2.goUp();

    do {
        pros::delay(10);
    } while (abs(forklift1.getError()) > 40);

    pros::delay(100);

    driveTrainPID.moveToPoint(Vector2( 0, 10 ), true);

    /*
    startIntakeSmoothMove(false, false);
    intake.counterClockwise(80);

    pros::delay(5000);

    driveTrainPID.move({}, 0);
    intake.stop();
    */
}

void testForklifts() {
    forklift1.goDown();
    forklift2.goDown();

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);

    forklift1.goUp();
    forklift2.goUp();

    // Wait for it to reach okay tolerance levels
    do {
        pros::delay(10);
    } while (abs(forklift2.getError()) > 40);
}

void myAuton() {
    trackingData.update({0, 0}, 0);
    // trackingData.update({0, 20}, 0);
    chassis->setState({0_in, 0_in, 90_deg});

    forklift1.enable();
    forklift2.enable();
    intake.enable();

    forklift1.goUp();
    forklift2.goUp();

    driveTrainPID.swingRotateTo(degToRad(45), RIGHT);
    driveTrainPID.rotateTo(degToRad(180));
    driveTrainPID.rotateTo(degToRad(50));

    // testForklifts();
    // grabMogo();
    
   // driveTrainPID.moveToPoint({ 0, 24 });
   // driveTrainPID.rotateTo(degToRad(180));

   /*
    tLeft.set_brake_mode(MOTOR_BRAKE_HOLD);
    tRight.set_brake_mode(MOTOR_BRAKE_HOLD);
    bLeft.set_brake_mode(MOTOR_BRAKE_HOLD);
    bRight.set_brake_mode(MOTOR_BRAKE_HOLD);

    PIDController platformController(0, PIDInfo(0.25, 0, 0), 2, 0);
    while (true) {
        // Get pitch
        double pitch = myImu.get_pitch();
        
        driveTrainPID.move(platformController.step(pitch), 0);

        pros::delay(10);
    }
    */
}