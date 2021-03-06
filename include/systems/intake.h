/**
 * \file intake.h
 * 
 * \brief Contains headers for the Intake system manager.
*/

#pragma once

#include "main.h"
#include "systemManager.h"
#include "chassis.h"
#include "control/PID.h"

#define INTAKE_HOLD_SPEED 80

/**
 * \brief Intake system manager class, inherits from SystemManager. 
*/
class Intake: public SystemManager {
    public:
        /**
         * All possible states for the system.
        */
        static const uint8_t DISABLED_STATE = 0x00;
        static const uint8_t RESET_STATE = 0x01;
        static const uint8_t CLOCKWISE_STATE = 0x02;          // Intake going clockwise
        static const uint8_t COUNTER_CLOCKWISE_STATE = 0x03;  // Intake going counter clockwise
        static const uint8_t HOLD_STATE = 0x04;               // Hold intake in place (idle about don't allow freespin)
        static const uint8_t OPERATOR_OVERRIDE = 0x20;

        /**
         * Constructor for the Intake class
         * @param defaultState The default state for the system.
        */
        Intake(uint8_t defaultState, PIDInfo constants);

        /**
         * Set the new power for the intake.
         * @param newPower The new power level
        */
        void setPower(double newPower);

        /**
         * Move the motor for the intake clockwise.
         * @param newPower The power at which to run the intake
        */
        void clockwise(double newPower);

        /**
         * Move the motor for the intake counter-clockwise.
         * @param newPower The power at which to run the intake
        */
        void counterClockwise(double newPower);

        /**
         * Allow for an operator to control the system.
        */
        void control();

        /**
         * Stop the intake.
        */
        void stop();

        /**
         * The control loop for the system. This should be run every loop.
        */
        void update() override;

        /**
         * Run a full reset of the system and its variables and state.
        */
        void fullReset() override;

        // Intake motor
        pros::Motor intakeMotor = pros::Motor(INTAKE_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

    protected:
        /**
         * Sets the state of the system.
         * @param newState The new state of the system.
         * @return True if the system was enabled, and false if it was disabled.
        */
        virtual bool changeState(uint8_t newState);

        /**
         * Checks whether the last change of state was made during a specific period of time.
         * @param timeout The amount of milliseconds after the last state change.
         * @return True if the system state was changed before the time period, False if not.
        */
        bool timedOut(uint32_t timeout);

        PIDInfo constants;

        PIDController *intake_pidController;
};
