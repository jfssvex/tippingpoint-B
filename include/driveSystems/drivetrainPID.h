/**
 * \file drivetrainPID.h
 * 
 * \brief Contains headers for the Drivetrain PID class which serves as a wrapper class on top of the Drivetrain class to use PID + Odometry.
*/

#pragma once

#include "main.h"
#include "drivetrain.h"
#include "control/PID.h"
#include "tracking.h"

/**
 * \brief Wrapper class on top of Drivetrain class to implement PID + Odom on any drivetrain
*/
class DrivetrainPID {
    public:
        /**
         * Initializes the Drivetrain class
         * @param drivetrain The type of drivetrain (ex. SkidSteerDrive) used
         * @param driveConstants The PID gain constants for the drive PID controller
         * @param turnConstants The PID gain constants for the turn PID controller
         * @param distTolerance The tolerance for the distance controller before it can be considered settled
         * @param distIntegralTolerance The threshold at which the distance integral will be used or not
         * @param turnTolerance The tolerance for the turn controller before it can be considered settled
         * @param turnIntegralTolerance The threshold at which the turn integral will be used or not
        */
        DrivetrainPID(Drivetrain* drivetrain, PIDInfo driveConstants, PIDInfo turnConstants, double distTolerance, double distIntegralTolerance, double turnTolerance, double turnIntegralTolerance);

        /**
         * Destructor for class
        */
        ~DrivetrainPID();
        
        /**
         * Set the velocity based on the current and wanted position and angle
         * @param dir The direction as a Vector2
         * @param turn The angle in radians
        */
        void move(Vector2 dir, double turn, bool backwards = false);

        /**
         * Turn to the angle needed to reach a position, drive to the position, and then turn to the desired angle
         * @param target The position to reach as a Vector2
         * @param angle The angle desired at the end of the action in radians
        */
        void moveToOrientation(Vector2 target, double angle);

        /**
         * Move to a specific point on the field
         * @param target The position to reach as a Vector2
        */ 
        void moveToPoint(Vector2 target, bool backwards = false);

        /**
         * Move to a specific point without turning on the spot first
         * @param target The position to reach as a Vector2
        */
        void experimentalMoveToPoint(Vector2 target);

        /**
         * Move and turn to a specific point and orientation relative to the bot's current
         * position and orientation
         * @param offset The desired position relative to the robot's current position as a Vector2
         * @param aOffset The desired angle relative to the robot's current orientation in radians
        */
        void moveRelative(Vector2 offset, double aOffset);

        /**
         * Rotate the robot to the desired orientation
         * @param angle The desired rotation in radians
        */ 
        void rotateTo(double angle);

        /**
         * Returns the drive controller
        */ 
        PIDController* getDriveController() { return driveController; };

        /**
         * Returns the turn controller
        */ 
        PIDController* getTurnController() { return turnController; };
    
    private:
        // PID Drive Controller
        PIDController* driveController;

        // PID Turn Controller
        PIDController* turnController;

        // Pointer to drivetrain, note that it must refer to a derrived class
        Drivetrain* drivetrain;
};