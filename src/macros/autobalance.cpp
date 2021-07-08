#include "macros.h"
#include "globals.h"
#include "main.h"
#include "control/PID.h"

void autoBalanceRobot() {
    // Move the drivetrain to be completely on the platform
    driveTrainPID.moveToOrientation(Vector2(2, 0), degToRad(90));

    // Create a PID controller for the pitch
    PIDController pitchController = PIDController(
        0, // It should aim to be flat (0 degrees)
        PIDInfo(1, 1, 0), // PI Controller for now
        1,
        0.3
    );

    // TODO: Fix this to actually work, will probably require some testing
    while (!pitchController.isSettled()) {
        double vel = pitchController.step(degToRad(imuSensor.get_pitch()));
        driveTrainPID.move(Vector2(vel, 0), 0);

        pros::delay(20); 
    }
}