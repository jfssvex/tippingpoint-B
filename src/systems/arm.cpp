#include "globals.h"
#include "serialLogUtil.h"
//Motor definition in arm.h

Arm::Arm(uint8_t defaultState, double upPos, double downPos, PIDInfo constants) : SystemManager(defaultState) {
    this->defaultState = defaultState;
    this->upPos = upPos;
    this->downPos = downPos;

    this->constants = constants;
    this->pidController = new PIDController(10, this->constants, 10, 1);

    armMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
}

Arm::~Arm() {
    delete this->pidController;
}

void Arm::setPower(int armPower) {
    armManualPower = armPower;
    armMotor.move(armPower);
}

void Arm::raise() {
    this->changeState(UP_STATE);
}

void Arm::lower() {
    this->changeState(DOWN_STATE);
}

void Arm::control() {
    this->changeState(OPERATOR_OVERRIDE);
}

int adustMotorPIDOutput(double inp) {
    if (abs(inp) > 127) {
        int inpNeg = inp < 0 ? -1 : 1;
        return 127 * inpNeg;
    }
    return floor(inp);
}

void Arm::update() {
    this->position = armMotor.get_position();   
    
    if (armManualPower == 0) {
        // Retain position if manual power not being applied with custom PID loop
        double speed = pidController->step(this->position);

        if (abs(speed) >= 5) {
            armMotor.move(-adustMotorPIDOutput(speed));
            this->power = -adustMotorPIDOutput(speed); //idk what power is for
        } else {
            armMotor.move(0);
            this->power = 0; // here too
        }

    } else {
        // Update target to be current position
        this->pidController->target = this->position;
    }
    // Set internal variables to match controller
    this->error = pidController->getError();
    this->target = pidController->target;
}

bool Arm::changeState(uint8_t newState) {
    // Run basic state change code in base function
    bool processed = SystemManager::changeState(newState);

    if (!processed) {
        colorPrintf("State change not processed!\n", RED);
        return false;
    }

    switch(newState) {
        case UP_STATE: {
            // Set forklift motor to hold at up position
            armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->pidController->target = upPos;
            break;
        }
        case DOWN_STATE: {
            // Set forklift motor to hold at down position
            armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->pidController->target = downPos;
            break;
        }
        case DISABLED_STATE: {
            // Stop motors & leave limp
            armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            armMotor = 0;
            break;
        }
        case OPERATOR_OVERRIDE: {
            armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

            // Set target to current position since user will be controlling anyways
            this->pidController->target = armMotor.get_position();
            break;
        }
    }

    colorPrintf("NEW TARGET: %f\n", GREEN, this->pidController->target);

    return true;
}

void Arm::fullReset() {
    armMotor.tare_position();
    SystemManager::fullReset();
}
