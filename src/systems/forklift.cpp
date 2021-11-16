/**
 * Implementation for the Intake system manager.
*/

#include "systems/forklift.h"
#include "systems/systemManager.h"

Forklift::Forklift(STATE defaultState, pros::Motor* forkliftMotor) : SystemManager((SystemManager::STATE) defaultState) {
    this->defaultState = defaultState;
    this->forkliftMotor = forkliftMotor;
}

void Forklift::goUp() {
    this->changeState(UP_STATE);
}

void Forklift::goDown() {
    this->changeState(DOWN_STATE);
}

Forklift::STATE Forklift::getState() {
    return this->state;
}

// TODO: Add custom PID loop
void Forklift::update() {
    switch(this->state) {
        case UP_STATE:
            break;
        case DOWN_STATE:
            break;
        case RESET_STATE: 
            break;
        case DISABLED_STATE:
            break;
    }
}

bool Forklift::changeState(STATE newState) {
    // Run basic state change code in base function
    bool processed = SystemManager::changeState((SystemManager::STATE) newState);

    if (!processed) {
        printf("dead\n");
        return false;
    }

    printf("not dead\n");

    switch(newState) {
        case UP_STATE: {
            // Set forklift motor to hold at up position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->forkliftMotor->move_relative(360, 100);
            // this->forkliftMotor->move_absolute(120, 100); // TODO: Change to maximum rpm based on cartridge
            break;
        }
        case DOWN_STATE: {
            // Set forklift motor to hold at down position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->forkliftMotor->move_absolute(0, 100); // TODO: Change to maximum rpm based on cartridge
            break;
        }
        case DISABLED_STATE: {
            // Stop motors & leave limp
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            this->forkliftMotor = 0;
            break;
        }
        case OPERATOR_OVERRIDE: {
            // Allow motor to coast
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            break;
        }
    }

    return true;
}

void Forklift::fullReset() {
    this->forkliftMotor->tare_position();
    SystemManager::fullReset();
}