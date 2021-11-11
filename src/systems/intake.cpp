/**
 * Implementation for the Intake system manager.
*/

#include "systems/intake.h"
#include "systems/systemManager.h"

double abs(double x) {
    return x >= 0 ? x : -x;
}

Intake::Intake(STATE defaultState) : SystemManager((SystemManager::STATE) defaultState) {
    this->defaultState = defaultState;
}


void Intake::setPower(double newPower) {
    this->power = newPower;

    // Set new power to motor
    intakeMotor.move(this->power);
}

void Intake::clockwise(double newPower) {
    this->power = abs(newPower);
    this->changeState(CLOCKWISE_STATE);
}

void Intake::counterClockwise(double newPower) {
    this->power = -abs(newPower);
    this->changeState(COUNTER_CLOCKWISE_STATE);
}

void Intake::control() {
    this->changeState(OPERATOR_OVERRIDE);
}

void Intake::stop() {
    this->changeState(HOLD_STATE);
}

void Intake::update() {
    // Update PID variables
    this->position = this->intakeMotor.get_position();
    this->error = this->position - this->target;

    switch(this->state) {
        case CLOCKWISE_STATE:
            break;
        case COUNTER_CLOCKWISE_STATE:
            break;
        case HOLD_STATE: {
            // Keep intake in same position at all times
            if (abs(this->error) >= 40) {
                this->intakeMotor.move_absolute(0, INTAKE_HOLD_SPEED);
            }
            break;
        }
        case RESET_STATE: 
            break;
        case DISABLED_STATE:
            break;
    }
}

bool Intake::changeState(STATE newState) {
    // Run basic state change code in base function
    bool processed = SystemManager::changeState((SystemManager::STATE) newState);

    if (!processed) {
        return false;
    }

    switch(newState) {
        case CLOCKWISE_STATE: {
            // Run intake clockwise
            this->intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            this->setPower(this->power);
            break;
        }
        case COUNTER_CLOCKWISE_STATE: {
            // Run intake counter-clockwise
            this->intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            this->setPower(this->power);
            break;
        }
        case HOLD_STATE: {
            // Set brakes to hold at exact position
            this->intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

            // Turn off power to motor
            this->setPower(0);

            // Call move_absolute to make sure it actually does hold
            this->intakeMotor.tare_position();
            this->target = 0;
            this->intakeMotor.move_absolute(0, INTAKE_HOLD_SPEED);
            break;
        }
        case DISABLED_STATE: {
            // Stop motors & leave limp
            this->intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            this->setPower(0);
            break;
        }
        case OPERATOR_OVERRIDE: {
            // Allow motor to coast
            this->intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            break;
        }
    }

    return true;
}

void Intake::fullReset() {
    this->intakeMotor.tare_position();
    SystemManager::fullReset();
}