#include "globals.h"

Intake intake(Intake::HOLD_STATE);
Forklift forklift1(Forklift::DOWN_STATE, forklift1Motor, PIDInfo(1, 1, 1), 0, 0);
Forklift forklift2(Forklift::DOWN_STATE, forklift2Motor, PIDInfo(1, 1, 1), 0, 0);
//TODO: real values for k constants and sort out proper method of handling totalError and lastError