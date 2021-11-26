/**
 * Implementation for the Intake system manager.
*/

#include "systems/forklift.h"
#include "systems/systemManager.h"

Forklift::Forklift(uint8_t defaultState, pros::Motor* forkliftMotor, PIDInfo constants) : SystemManager(defaultState) {
    this->defaultState = defaultState;
    this->forkliftMotor = forkliftMotor;

    this->constants = constants;
    this->pidController = new PIDController(0, this->constants, 10, 1);

    forkliftMotor->set_brake_mode(MOTOR_BRAKE_HOLD);
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
        // Retain position if manual power not being applied with custom PID loop
        double speed = pidController->step(this->forkliftMotor->get_position());
        this->forkliftMotor->move(speed);
    } else {
        // Update target to be current position
        this->pidController->target = this->forkliftMotor->get_position();
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