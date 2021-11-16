/**
 * \file forklift.h
 * 
 * \brief Contains headers for the Intake system manager.
*/

#pragma once

#include "main.h"
#include "systemManager.h"
#include "chassis.h"

/**
 * \brief Forklift system manager class, inherits from SystemManager. 
*/
class Forklift: public SystemManager {
    public:
        /**
         * All possible states for the system.
        */
        enum STATE {
            DISABLED_STATE = (uint8_t) 0x00,
            RESET_STATE = (uint8_t) 0x01,
            UP_STATE = (uint8_t) 0x02,          // Forklift holding MOGO up
            DOWN_STATE = (uint8_t) 0x03,        // Forklift holding MOGO down
            OPERATOR_OVERRIDE = (uint8_t) 0x20
        };

        /**
         * Constructor for the Forklift class
         * @param defaultState The default state for the system.
        */
        Forklift(STATE defaultState, pros::Motor* forkliftMotor);

        /**
         * Bring the forklift up
        */
        void goUp();

        /**
         * Bring the forklift down
        */
        void goDown();

        /**
         * Allow for an operator to control the system.
        */
        void control();

        /**
         * Revert the state to what it previously was.
        */
        void revertState();

        /**
         * Get the current state.
        */
        Forklift::STATE getState();

        /**
         * The control loop for the system. This should be run every loop.
        */
        void update() override;

        /**
         * Run a full reset of the system and its variables and state.
        */
        void fullReset() override;

    protected:
        /**
         * The previous state of the system.
        */
        STATE lastState = DISABLED_STATE;

        /**
         * The current state of the system.
        */
        STATE state = DISABLED_STATE;

        /**
         * The default state of the system.
        */
        STATE defaultState;

        /**
         * Sets the state of the system.
         * @param newState The new state of the system.
         * @return True if the system was enabled, and false if it was disabled.
        */
        virtual bool changeState(STATE newState);

        /**
         * Checks whether the last change of state was made during a specific period of time.
         * @param timeout The amount of milliseconds after the last state change.
         * @return True if the system state was changed before the time period, False if not.
        */
        bool timedOut(uint32_t timeout);

        // Forklift motor
        pros::Motor* forkliftMotor;
};