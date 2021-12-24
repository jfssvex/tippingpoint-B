#include "macros.h"
#include "globals.h"

const int DRIVETRAIN_FORWARD_SPEED = 35; // Speed at which the drivetrain should go forward, TODO finetune this
const int INTAKE_CC_SPEED = 85; // Speed at which the intake should go clockwise, finetune this

void startIntakeSmoothMove(bool driveBack, bool intakeCCW) {
    // Drive backwards if bool is set, otherwise drive forwards
    driveTrain->forward(driveBack ? -DRIVETRAIN_FORWARD_SPEED : DRIVETRAIN_FORWARD_SPEED);
}

void stopIntakeSmoothMove() {
    // driveTrain->stop();
    // intake.stop();
}
