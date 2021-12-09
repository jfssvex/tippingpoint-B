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
#define FORKLIFT_2_PORT 10

// Encoder pin numbers
#define BENC_PORT_TOP 5
#define LENC_PORT_TOP 6
#define RENC_PORT_TOP 7

#define BENC_PORT_BOT 8
#define LENC_PORT_BOT 9
#define RENC_PORT_BOT 10

// Robot dimensions in inches (TODO: Update to real dimensions)
#define DRIVE_WHEEL_DIAMETER 3.25f // ! MEASURE THIS!!!
#define TRACKING_WHEEL_DIAMETER 2.75f
#define WHEELBASE 10.25f // ! MEASURE THIS!!!
#define BACK_WHEEL_OFFSET 5.0f