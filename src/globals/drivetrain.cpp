#include "globals.h"
#include "chassis.h"

// PID Gain Constants
PIDInfo driveConstants(0.5, 0.01, 0.005);
PIDInfo driveConstantsOkapi(0.005, 0.0001, 0.00005);
PIDInfo turnConstants(0.1, 0, 0.25);

// Definitions
SkidSteerDrive* driveTrain = new SkidSteerDrive(&tLeft, &tRight, &bLeft, &bRight);


DrivetrainPID driveTrainPID(
    driveTrain, 
    driveConstants, 
    turnConstants, 
    0.7,  // Inches
    1,    // Inches
    1,    // Degrees
    0.3   // Degrees
);


std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(
		{TL_PORT, BL_PORT}, // Left motors are 1 & 2 (reversed)
        {TR_PORT, BR_PORT}    // Right motors are 3 & 4
	) 
    .withGains(
        {driveConstantsOkapi.p, driveConstantsOkapi.i, driveConstantsOkapi.d}, // Distance controller gains
        {0.001, 0, 0.0001}, // Turn controller gains
        {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
    // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 14_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis