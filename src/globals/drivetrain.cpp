#include "globals.h"
#include "chassis.h"

// PID Gain Constants
PIDInfo driveConstants(1, 1, 1);
PIDInfo turnConstants(1, 0, 0);

// Definitions
SkidSteerDrive* driveTrain = new SkidSteerDrive(&tLeft, &tRight, &bLeft, &bRight);
DrivetrainPID driveTrainPID(driveTrain, driveConstants, turnConstants, 0, 0);