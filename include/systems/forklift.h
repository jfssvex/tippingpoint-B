/**
 * \file forklift.h
 * 
 * \brief Contains headers for the Intake system manager.
*/

#pragma once

#include "main.h"
#include "systemManager.h"
#include "chassis.h"
#include "control/PID.h"

/**
 * \brief Forklift system manager class, inherits from SystemManager. 
*/
class Forklift: public SystemManager {
    public:
        /**
         * All possible states for the system.
        */
        static const uint8_t DISABLED_STATE = 0x00;
        static const uint8_t RESET_STATE = 0x01;
        static const uint8_t UP_STATE = 0x02;            // Forklift holding MOGO up
        static const uint8_t MIDDLE_STATE = 0x04;        // Forklift holding MOGO for transport
        static const uint8_t DOWN_STATE = 0x03;          // Forklift holding MOGO down
        static const uint8_t OPERATOR_OVERRIDE = 0x20;
        
        /**
         * Constructor for the Forklift class
         * @param defaultState The default state for the system.
        */
        Forklift(uint8_t defaultState, pros::Motor* forkliftMotor, PIDInfo constants);

        /**
         * Bring the forklift up
        */
        void goUp();

        /**
         * Bring the forklift into the middle position
        */
        void goMiddle();

        /**
         * Bring the forklift down
        */
        void goDown();

        /**
         * Set the power of the forklift motor
        */
        void setPower(int power);

        /**
         * Allow for an operator to control the system.
        */
        void control();

        /**
         * The control loop for the system. This should be run every loop.
        */
        void update() override;

        /**
         * Run a full reset of the system and its variables and state.
        */
        void fullReset() override;

        /**
         * Sets the state of the system.
         * @param newState The new state of the system.
         * @return True if the system was enabled, and false if it was disabled.
        */
        virtual bool changeState(uint8_t newState);

    protected:
        // Forklift motor
        pros::Motor* forkliftMotor;

        // Manual power
        int manualPower = 0;

        PIDInfo constants;

        PIDController *pidController;
};