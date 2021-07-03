#include "macros.h"
#include "globals.h"
#include "main.h"
#include "control/PID.h"

void autoBalanceRobot() {
    // Move the drivetrain to be completely on the platform
    driveTrainPID.moveToPoint(Vector2(2, 0));

    // Create a PID controller for the pitch
    PIDController pitchController = PIDController(
        0, // It should aim to be flat (0 degrees)
        PIDInfo(1, 1, 0), // PI Controller for now
        1,
        0.3
    );

    // Stop until its settled
    while (!pitchController.isSettled()) {
        // Run a step of the controller with the new IMU pitch value
        double vel = pitchController.step(imuSensor.get_pitch());

        // Set the motors to the new vel
        driveTrainPID.move(Vector2(vel, 0), 0);

        pros::delay(20); 
    }
}