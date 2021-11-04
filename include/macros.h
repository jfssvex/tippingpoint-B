/**
 * \file intakeSmoothMove.h
 * 
 * \brief Contains headers for the IntakeSmoothMove macro.
*/

/**
 * Start the smooth intake movement. Basically runs the drivetrain
 * and the intake at a very specific speed which allows for
 * most optimal capturing of rings. 
 * @param driveBack Whether the drivetrain should go backward instead of forward.
 * @param intakeCCW Whether the intake should run counter-clockwise instead of counterclockwise.
*/
void startIntakeSmoothMove(bool driveBack, bool intakeCCW);

/**
 * Stops the macro.
*/
void stopIntakeSmoothMove();