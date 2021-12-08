#include "globals.h"
#include "chassis.h"

// PID Gain Constants
PIDInfo driveConstants(0.2, 0.000001, 0.1);
PIDInfo turnConstants(1.5, 0.1, 0.5);

// Definitions
SkidSteerDrive* driveTrain = new SkidSteerDrive(&tLeft, &tRight, &bLeft, &bRight);
DrivetrainPID driveTrainPID(
    driveTrain, 
    driveConstants, 
    turnConstants, 
    0.7,  // Inches
    3,    // Inches
    0.04, // Radians
    0.3   // Radians
);