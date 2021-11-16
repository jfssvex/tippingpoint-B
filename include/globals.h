/**
 * \file globals.h
 * 
 * \brief Contains all the global variables used throughout the project.
 * 
 * Contains global variables such as motors and encoders 
 * that are used throughout the project.
*/

#pragma once

#include "main.h"
#include "driveSystems/SkidSteerDrive.h"
#include "driveSystems/drivetrainPID.h"
#include "displayController.h"
#include "tracking.h"
#include "systems/intake.h"
#include "systems/forklift.h"

// Motors

// Drivetrain motors
extern pros::Motor tLeft; // Top left motor      
extern pros::Motor tRight; // Top right motor      
extern pros::Motor bLeft; // Bottom left motor
extern pros::Motor bRight; // Bottom right motor

// Forklift motors
extern pros::Motor* forklift1Motor; // Forklift 1 motor
extern pros::Motor* forklift2Motor; // Forklift 2 motor

// Encoders

// Drivetrain encoders
extern pros::ADIEncoder bEnc; // Back encoder
extern pros::ADIEncoder lEnc; // Left encoder
extern pros::ADIEncoder rEnc; // Right encoder

// System Managers
extern Intake intake;
extern Forklift forklift1;
extern Forklift forklift2;

// Drivetrain
extern SkidSteerDrive* driveTrain;
extern DrivetrainPID driveTrainPID;

// Odometry tracking
extern TrackingData trackingData;

// Display controller
extern DisplayController display;

// Actual controller
extern pros::Controller masterController;