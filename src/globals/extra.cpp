#include "globals.h"
#include "chassis.h"

// Display controller
DisplayController display = DisplayController();

// IMU (Inertial unit)
pros::Imu imuSensor(IMU_PORT);