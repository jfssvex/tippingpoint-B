#include "drivetrainPID.h"
#include "control/PID.h"
#include "tracking.h"
#include "globals.h"
#include "serialLogUtil.h"
#include <math.h>

#define flipAngle(a)  (a > 0) ? (-2 * M_PI + a) : (2 * M_PI + a) 
#define VECTOR_LENGTH(vec) sqrt(pow(vec.getX(), 2) + pow(vec.getY(), 2))
#define inchToCm(inch) inch*2.54
#define MAX_ACCELERATION 1

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

void stepMotor(pros::Motor motor, float targetSpeed) {
    float targetRPM = motor.get_target_velocity();
    auto gearset = motor.get_gearing();
    float rpmScale = 0;

    switch(gearset) {
        case (pros::E_MOTOR_GEARSET_36): { // Torque cartridge
            rpmScale = 100;
            break;
        }
        case (pros::E_MOTOR_GEARSET_18): { // High speed cartridge
            rpmScale = 200;
            break;
        }
        case (pros::E_MOTOR_GEARSET_06): { // Turbo cartridge
            rpmScale = 600;
            break;
        }
    }

    float pastTarget = (targetRPM / rpmScale) * 127;
    
    float delta = targetSpeed - pastTarget;
    if (abs(delta) > MAX_ACCELERATION) {
        delta = (delta / abs(delta)) * MAX_ACCELERATION;
    }

    motor.move(delta);
}

void DrivetrainPID::move(double straight, double turn) {
    // Get outputs for each side 
    double leftOutput = straight + turn;
    double rightOutput = straight - turn;

    // Scale to be between [-1, 1] if not
    double scalar = std::max(std::abs(leftOutput), std::abs(rightOutput));
    if (scalar > 1) {
        leftOutput /= scalar;
        rightOutput /= scalar;
    }

    // Scale motor velocities to be between [-127, 127]
    double leftMotorVel = leftOutput * 127;
    double rightMotorVel = rightOutput * 127;

    colorPrintf("Motor powers: %f %f\n", MAGENTA, rightMotorVel, rightMotorVel);
    
    // Set motor vel
    driveTrain->tank(leftMotorVel, rightMotorVel);
}

void DrivetrainPID::moveToOrientation(Vector2 target, double angle) {
    // Turn to angle and drive to position
    this->moveToPoint(target);

    // Turn to desired angle
    this->rotateTo(angle);
}

void DrivetrainPID::moveToPoint(Vector2 target, bool backwards) {
    // Turn to angle of point first (important in nonholonomic)
    Vector2 displacement = (target - trackingData.getPos());

    colorPrintf("Current position: %f %f\n", BLUE, trackingData.getPos().getX(), trackingData.getPos().getY());
    colorPrintf("Desired position: %f %f\n", BLUE, target.getX(), target.getY());
    colorPrintf("Displacement: %f %f\n", BLUE, displacement.getX(), displacement.getY());
    colorPrintf("\n\n\n----- ANGLE TO TURN TO: %f -----\n\n\n", BLUE, radToDeg(displacement.getAngle()));
    
    /*
    if (backwards) {
        this->rotateTo(-displacement.getAngle() + degToRad(90) + degToRad(180));
    } else {
        this->rotateTo(-displacement.getAngle() + degToRad(90));
    }
    

    pros::delay(2000);
    */

    // Get starting time
    double time = pros::millis();

    this->driveController->reset();
    this->driveController->target = 0; // Set target to 0 as loop will use delta as sense

    do {
        double deltaDist = VECTOR_LENGTH(target) - VECTOR_LENGTH(trackingData.getPos());

        if (trackingData.getPos().getX() < 0 || trackingData.getPos().getY() < 0) {
            deltaDist = -deltaDist;
        }

        // Vector2 delta = target - trackingData.getPos(); // This could also work buts it's very finicky in the simulator
        // double deltaDist = delta.getMagnitude();
        // double deltaDist = target.getX() - trackingData.getPos().getY();
        colorPrintf("Delta distance: %f\n", GREEN, deltaDist);

        // Get velocity from controller
        float vel = -(this->driveController->step(deltaDist));

        // Keep between [-1, 1]
        if (abs(vel) > 1) {
            vel /= abs(vel);
        }

        colorPrintf("Dist err: %f\nDist vel: %f\n\n", BLUE, deltaDist, vel);

        move(vel, 0);

        if (pros::millis() - time >= 3000) {
            // Taking too long, something might be going wrong
            break;
        }

        pros::delay(20);
    } while (!this->driveController->isSettled());

    move(0, 0);
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
        move(0, -turnController->step(trackingData.getHeading()));

        pros::delay(20);
    } while (!turnController->isSettled() && pros::millis() - time <= 3000); // Break if settled or taking more than 3s

    move(0, 0);
    trackingData.setAngleModulusSuspend(false);
}

double dot(double x1, double y1, double x2, double y2) {
    return (x1 * x2) + (y1 * y2);
}

Vector2 closest(Vector2 current, Vector2 target) {
    Vector2 head(sin(current.getAngle()), cos(current.getAngle()));
    
    Vector2 n = head.normalize();
    Vector2 v = target - current;
    double d = dot(v.getX(), v.getY(), n.getX(), n.getY());
    return current + (n * d);
}

double rollAngle180(double angle) {
    angle = radToDeg(angle);
    double newAngle = angle - 360 * std::floor((angle + 180.0) * (1.0 / 360.0));
    return degToRad(newAngle);
}

double rollAngle90(double angle) {
  angle = rollAngle180(angle);
  if (abs(angle) > degToRad(90)) {
    angle += degToRad(180);
    angle = rollAngle180(angle);
  }
  return angle;
}

void DrivetrainPID::experimentalMoveToPoint(Vector2 target) {
    driveController->reset();
    turnController->reset();

    double angleErr = 0, distanceErr = 0;

    do {
        Vector2 closestPoint = closest(trackingData.getPos(), target);

        Vector2 closestPointDisplacement = closestPoint - trackingData.getPos();
        Vector2 targetDisplacement = target - trackingData.getPos();

        double angleToClose = closestPoint.getAngle() - trackingData.getHeading();
        double angleToTarget = target.getAngle() - trackingData.getHeading();

        double distanceToClose = closestPointDisplacement.getMagnitude();
        double distanceToTarget = targetDisplacement.getMagnitude();

        // go backwards
        if (abs(angleToClose) >= degToRad(90)) distanceToClose = -distanceToClose;

        if (distanceToTarget < 1.5) {
            // Don't focus on angle adjustment
            angleErr = 0;
            // used for settling
            distanceErr = distanceToClose;
        } else {
            angleErr = angleToTarget;
            // used for settling
            distanceErr = distanceToTarget;
        }

        colorPrintf("Angle Err: %f\nDistance Err: %f\n\n", BLUE, angleErr, distanceErr);

        // rotate angle to be +- 90
        angleErr = rollAngle90(angleErr);

        double angleVel = turnController->step(-radToDeg(angleErr));
        double distanceVel = driveController->step(-inchToCm(distanceErr) * 10);

        move(distanceVel, 0);
        pros::delay(10);
    } while (!(driveController->isSettled() && turnController->isSettled()));

    move(0, 0);
}