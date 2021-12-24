/**
 * Implementation for the Intake system manager.
*/

#include "systems/forklift.h"
#include "systems/systemManager.h"
#include "serialLogUtil.h"

Forklift::Forklift(uint8_t defaultState, pros::Motor* forkliftMotor, PIDInfo constants, uint8_t POT_PORT, double downPos, double middlePos, double upPos) : SystemManager(defaultState) {
    this->defaultState = defaultState;
    this->forkliftMotor = forkliftMotor;

    this->constants = constants;
    this->pidController = new PIDController(0, this->constants, 0, 1); // No tolerance, always keep running

    this->potentiometer = new pros::ADIAnalogIn(POT_PORT);

    this->downPos = downPos;
    this->middlePos = middlePos;
    this->upPos = upPos;

    forkliftMotor->set_brake_mode(MOTOR_BRAKE_HOLD);

    // potentiometer->calibrate();
}

Forklift::~Forklift() {
    delete this->pidController;
    delete this->potentiometer;
    delete this->forkliftMotor;
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

void Forklift::calibrate() {
    this->potentiometer->calibrate();
    this->forkliftMotor->tare_position();
}

void Forklift::setPower(int power) {
    manualPower = power;
    this->forkliftMotor->move(power);
}

int scaleMotorPIDOutput(double inp) {
    if (abs(inp) > 127) {
        int inpNeg = inp < 0 ? -1 : 1;
        return 127 * inpNeg;
    }
    return floor(inp);
}

//TODO: tune PID constants
void Forklift::update() {
    this->position = this->potentiometer->get_value();    

    if (manualPower == 0) {
        // Retain position if manual power not being applied with custom PID loop
        double speed = pidController->step(this->position);
        this->forkliftMotor->move(scaleMotorPIDOutput(speed));
    } else {
        // Update target to be current position
        this->pidController->target = this->position;
    }

    colorPrintf("Forklift err: %f\n", BLUE, pidController->getError());
    colorPrintf("Forklift target: %f\n", RED, target);
    colorPrintf("Forklift sense: %f\n\n", RED, position);
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
            this->target = upPos;
            break;
        }
        case MIDDLE_STATE: {
            // Set forklift motor to hold at middle position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->target = middlePos;
            break;
        }
        case DOWN_STATE: {
            // Set forklift motor to hold at down position
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            this->target = downPos;
            break;
        }
        case DISABLED_STATE: {
            // Stop motors & leave limp
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            this->forkliftMotor = 0;
            break;
        }
        case OPERATOR_OVERRIDE: {
            // Only temporarily set to coast
            // TODO: Set back to hold
            this->forkliftMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            // Set target to current position since user will be controlling anyways
            this->target = this->potentiometer->get_value();

            break;
        }
    }

    colorPrintf("NEW TARGET: %f\n", GREEN, target);

    return true;
}

void Forklift::fullReset() {
    this->forkliftMotor->tare_position();
    SystemManager::fullReset();
}