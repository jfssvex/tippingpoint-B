#include "systems/arm.h"
//Motor definition in arm.h

Arm::Arm(uint8_t defaultState, double upPos, double downPos) : SystemManager(defaultState) {
    this->defaultState = defaultState;
    this->upPos = upPos;
    this->downPos = downPos;
}

void Arm::setPower(int armPower) {
    armManualPower = armPower;
    armMotor.move(armPower);
}

void Arm::control() {
    this->changeState(OPERATOR_OVERRIDE);
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

void Arm::fullReset() {
    armMotor.tare_position();
    SystemManager::fullReset();
}
