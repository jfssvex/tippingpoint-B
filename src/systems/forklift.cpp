/**
 * Implementation for the Intake system manager.
*/

#include "include/systems/forklift.h"
#include "include/systems/systemManager.h"
#include "PID.h"

Forklift::Forklift(uint8_t defaultState, pros::Motor* forkliftMotor, PIDInfo constants, double totalError, double lastError) : SystemManager(defaultState) {
    this->defaultState = defaultState;
    this->forkliftMotor = forkliftMotor;
    this->constants = constants;
    this->totalError = totalError;
    this->lastError = lastError;
}

void Forklift::goUp() {
    this->changeState(UP_STATE);
}

void Forklift::goMiddle() {
    this->changeState(MIDDLE_STATE);
}

void Forklift::goDown() {
    this->changeState(DOWN_STATE);
}

void Forklift::control() {
    this->changeState(OPERATOR_OVERRIDE);
}

void Forklift::setPower(int power) {
    manualPower = power;
    this->forkliftMotor->move(power);
}

// TODO: Add custom PID loop
void Forklift::update() {
    if (manualPower == 0) {
        // Retain position if manual power not being applied
        this->forkliftMotor->move_absolute(this->target, 200);
    } else {
        // Update target to be current position
        //this->target = this->forkliftMotor->get_position();
        
        // need way to get target
        int currValue = this->forkliftMotor->get_position();
        int currError = currValue - this->target;
        this->totalError += currError;
        int dError = currError - this->lastError;

        int currSpeed = (this->constants.p * currError) + (this->constants.i * this->totalError) + (this->constants.d * dError);

        this->forkliftMotor->move(currSpeed);

        this->lastError = currError;
    }
}

bool Forklift::changeState(uint8_t newState) {
    // Run basic state change code in base function
    bool processed = SystemManager::changeState(newState);

    if (!processed) {
        return false;
    }

    switch(newState) {
        case UP_STATE: {
            // Set forklift motor to hold at up position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->forkliftMotor->move_absolute(540, 200);
            break;
        }
        case MIDDLE_STATE: {
            // Set forklift motor to hold at middle position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->forkliftMotor->move_absolute(300, 200); 
            break;
        }
        case DOWN_STATE: {
            // Set forklift motor to hold at down position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->forkliftMotor->move_absolute(0, 200);
            break;
        }
        case DISABLED_STATE: {
            // Stop motors & leave limp
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            this->forkliftMotor = 0;
            break;
        }
        case OPERATOR_OVERRIDE: {
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            break;
        }
    }

    return true;
}

void Forklift::fullReset() {
    this->forkliftMotor->tare_position();
    SystemManager::fullReset();
}