#include "drivetrainPID.h"
#include "control/PID.h"
#include "tracking.h"
#include "globals.h"
#include "serialLogUtil.h"
#include <math.h>

// Flips radian angle
#define flipAngle(a)  (a > 0) ? (-2 * M_PI + a) : (2 * M_PI + a) 

#define VECTOR_LENGTH(vec) sqrt(pow(vec.getX(), 2) + pow(vec.getY(), 2))

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

    // Calculate distance using pythagorean theorem and motor velocity
    double distance = dir.getMagnitude();

    // Get outputs for each side 
    double leftOutput = distance + turn;
    double rightOutput = distance - turn;

    // Scale to be between [-1, 1] if not
    double scalar = std::max(std::abs(leftOutput), std::abs(rightOutput));
    if (scalar > 1) {
        leftOutput /= scalar;
        rightOutput /= scalar;
    }

    double leftMotorVel = leftOutput * 127;
    double rightMotorVel = rightOutput * 127;

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
    Vector2 displacement = (target - trackingData.getPos());

    colorPrintf("Current position: %f %f\n", BLUE, trackingData.getPos().getX(), trackingData.getPos().getY());
    colorPrintf("Desired position: %f %f\n", BLUE, target.getX(), target.getY());
    colorPrintf("Displacement: %f %f\n", BLUE, displacement.getX(), displacement.getY());
    colorPrintf("\n\n\n----- ANGLE TO TURN TO: %f -----\n\n\n", BLUE, radToDeg(displacement.getAngle()));

    this->rotateTo(-displacement.getAngle());

    // Get starting time
    double time = pros::millis();

    this->driveController->reset();
    this->driveController->target = 0; // Set target to 0 as loop will use delta as sense

    do {
        double deltaDist = VECTOR_LENGTH(target) - VECTOR_LENGTH(trackingData.getPos());
        // Vector2 delta = target - trackingData.getPos(); // This could also work buts it's very finicky in the simulator
        // double deltaDist = delta.getMagnitude();
        // double deltaDist = target.getX() - trackingData.getPos().getY();
        colorPrintf("Delta distance: %f\n", GREEN, deltaDist);

        if (deltaDist < 0.2) {
            deltaDist = 0;
        }

        // Flip positivity since we're using the delta as the sense
        float vel = -(this->driveController->step(deltaDist));
        colorPrintf("Dist err: %f\nDist vel: %f\n\n", BLUE, deltaDist, vel);

        // No need to include angle data since it's already at angle needed to move to position
        // Any issues? Divide vel by 2
        this->move({ 0, vel }, 0);


        if (pros::millis() - time >= 1500) {
            // Taking too long, something might be going wrong
            break;
        }

        pros::delay(20);
    } while (!this->driveController->isSettled());

    move({}, 0);
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

    // target = radToDeg(target);

    // Get starting time
    double time = pros::millis();

    // Turn the other way if it's more efficient
    if (abs(target - trackingData.getHeading()) > degToRad(180)) {
        target = flipAngle(target);
    }

    // turnController->reset();
    turnController->target = target;
    do {
        // Run PID step and move to angle
        move(Vector2(), -turnController->step(trackingData.getHeading()));

        pros::delay(20);
    } while (!turnController->isSettled() && pros::millis() - time <= 3000); // Break if settled or taking more than 3s

    move({}, 0);
    trackingData.setAngleModulusSuspend(false);
}