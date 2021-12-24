#include "globals.h"
#include "chassis.h"

// PID Gain Constants
PIDInfo driveConstants(0.2, 0.000001, 0.1);
PIDInfo turnConstants(1, 0, 0);

// Definitions
SkidSteerDrive* driveTrain = new SkidSteerDrive(&tLeft, &tRight, &bLeft, &bRight);
DrivetrainPID driveTrainPID(
    driveTrain, 
    driveConstants, 
    turnConstants, 
    0.7,  // Inches
    1,    // Inches
    degToRad(0), // Radians
    0.3   // Radians
);