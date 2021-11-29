#include "globals.h"

Intake intake(Intake::HOLD_STATE, PIDInfo(1, 1, 1));

// TODO: tune constants
Forklift forklift1(Forklift::DOWN_STATE, forklift1Motor, PIDInfo(1, 1, 1));
Forklift forklift2(Forklift::DOWN_STATE, forklift2Motor, PIDInfo(1, 1, 1));