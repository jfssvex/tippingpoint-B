/**
 * \file chassis.h
 * 
 * \brief Contains definition macros such as pin numbers.
 * 
 * Contains definition macros such as pin numbers for a variety of
 * components (ex. motors, encoders) and dimensions.
*/

#pragma once

#include "driveSystems/drivetrainPID.h"

// Motor pin numbers
#define TL_PORT 2
#define TR_PORT 3
#define BL_PORT 4
#define BR_PORT 5

#define INTAKE_PORT 12
#define FORKLIFT_1_PORT 11
#define FORKLIFT_2_PORT 13

// Encoder pin numbers
#define BENC_PORT_TOP 5
#define LENC_PORT_TOP 6
#define RENC_PORT_TOP 7

#define BENC_PORT_BOT 8
#define LENC_PORT_BOT 9
#define RENC_PORT_BOT 10

// Potentiometer pins
#define POT_PORT_1 'E'
#define POT_PORT_2 'F'

// Robot dimensions in inches (TODO: Update to real dimensions)
#define DRIVE_WHEEL_DIAMETER 3.25f
#define TRACKING_WHEEL_DIAMETER 2.75f
#define WHEELBASE 14
#define BACK_WHEEL_OFFSET 0