#include "drivetrainPID.h"
#include "control/PID.h"
#include "tracking.h"
#include "globals.h"
#include <math.h>

// Flips radian angle
#define flipAngle(a)  (a > 0) ? (-2 * M_PI + a) : (2 * M_PI + a) 

DrivetrainPID::DrivetrainPID(Drivetrain* drivetrain, PIDInfo driveConstants, PIDInfo turnConstants, double distTolerance, double distIntegralTolerance, double turnTolerance, double turnIntegralTolerance) {
    this->driveController = new PIDController(0, driveConstants, distTolerance, distIntegralTolerance);
    this->turnController = new PIDController(0, turnConstants, turnTolerance, turnIntegralTolerance);
    this->drivetrain = drivetrain;
}

DrivetrainPID::~DrivetrainPID() {
    delete this->driveController;
    delete this->turnController;
    delete this->drivetrain;
}

void DrivetrainPID::move(Vector2 dir, double turn) {
    dir = toLocalCoordinates(dir);
    double velX = dir.getX();
    double velY = dir.getY();

    // Calculate distance using pythagorean theorem and motor velocity
    double distance = dir.getMagnitude();

    // Scale to be between [-127, 127] if not
    double scalar = 1;
    double maxInput = std::max(std::abs(distance + turn), std::abs(distance - turn));
    if (maxInput > 1) {
        scalar = maxInput;
    }

    double leftMotorVel = (distance + turn) / scalar * 127;
    double rightMotorVel = (distance - turn) / scalar * 127;

    // Set motor vel
    driveTrain->tank(leftMotorVel, rightMotorVel);
}

void DrivetrainPID::moveToOrientation(Vector2 target, double angle) {
    // Turn to angle and drive to position
    this->moveToPoint(target);

    // Turn to desired angle
    this->rotateTo(angle);
}

void DrivetrainPID::moveToPoint(Vector2 target) {
    // Turn to angle of point first (important in nonholonomic)
    this->rotateTo(target.getAngle());

    // Get starting time
    double time = pros::millis();

    this->driveController->target = 0; // Set target to 0 as loop will use delta as sense

    do {
        Vector2 delta = target - trackingData.getPos();

        // Flip positivity since we're using the delta as the sense
        float vel = -(this->driveController->step(delta.getMagnitude()));

        // Rotate the vector to restore direction since it only points left or right as of now
        Vector2 driveVec = rotateVector(Vector2(vel, 0), delta.getAngle());

        this->move(driveVec, 0);

        pros::delay(20);
    } while (!this->driveController->isSettled() || pros::millis() - time <= 5000);
}

void DrivetrainPID::moveRelative(Vector2 offset, double aOffset) {
    // Get the desired absolute position & angle
    Vector2 desiredPos = trackingData.getPos() + offset;
    double desiredAngle = trackingData.getHeading() + aOffset;

    // Move robot to desired position & angle
    this->moveToOrientation(desiredPos, desiredAngle);
}

void DrivetrainPID::rotateTo(double target) {
    // Stop applying modulo to angle to prevent issues
    trackingData.setAngleModulusSuspend(true);

    // Get starting time
    double time = pros::millis();

    // Turn the other way if it's more efficient
    if (abs(target - trackingData.getHeading()) > degToRad(180)) {
        target = flipAngle(target);
    }

    turnController->target = target;
    do {
        // Run PID step and move to angle
        move(Vector2(), turnController->step(trackingData.getHeading()));

        pros::delay(20);
     while (!turnController->isSettled() || pros::millis() - time <= 3000); // Break if settled or taking more than 3s

    masterController.print(0, 0, "Done turning!");

    trackingData.setAngleModulusSuspend(false);
}