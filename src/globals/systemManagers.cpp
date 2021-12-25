#include "globals.h"

Intake intake(Intake::HOLD_STATE, PIDInfo(1, 1, 1));

// TODO: tune constants
Forklift forklift1(Forklift::DOWN_STATE, forklift1Motor, PIDInfo(0.5, 0.1, 0), POT_PORT_1, 1826, 1319, 824);
Forklift forklift2(Forklift::DOWN_STATE, forklift2Motor, PIDInfo(0.25, 0, 0), POT_PORT_2, 1840, 2282, 2983);

