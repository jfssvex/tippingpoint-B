#include "main.h"
#include "globals.h"
#include "macros.h"
#include "systems/intake.h"

// Thresholds a joystick value
#define thresholdJoystick(joystickVal, threshold) abs(joystickVal) > threshold ? joystickVal : 0

// Get a digital press, makes it short form for the utility
#define getDigitalPress(KEY) masterController.get_digital(KEY)

#define getNewDigitalPress(KEY) masterController.get_digital_new_press(KEY)

// For maintainance utility
#define getLeftJoystick() thresholdJoystick(left, threshold)

/**
 * \brief Scale joystick output to cubic graph
 * 
 * Scales raw value in range [-127, 127] to range [-1, 1] and 
 * apply exponent 3 in order to allow for more driver accuracy
*/
double joystickCubicDrive(int raw) {
    double scaledRaw = ((double)raw) / 127.0;
    return raw < 0 ? pow(scaledRaw, 2) * -127 : pow(scaledRaw, 2) * 127;
}

void myOpControl() {
    // Enable all systems
    intake.enable();
    forklift1.enable();
    forklift2.enable();

    // 0 -> nothing, 1 -> clockwise, -1 -> counter clockwise
    int macroToggle = 0;

    // Internal state for forklift 1
    // 0 -> Down, 1 -> Middle, 2 -> Up
    int forklift1State = 0;

    bool drivetrainHold = false;

    int fspeed = 127;
    int ispeed = 80;
    int tLeftSpeed = 0;
    int tRightSpeed = 0;
    int bLeftSpeed = 0;
    int bRightSpeed = 0;

    int maintenanceToggle = 0;

    while (true) {
        //on and off
        if (getNewDigitalPress(DIGITAL_A)) {
            maintenanceToggle = !maintenanceToggle;
        }

        // Arcade drive controls
        // int forward = masterController.get_analog(ANALOG_LEFT_Y);
        // int yaw = masterController.get_analog(ANALOG_RIGHT_X);

        // Tank drive controls
        int left = masterController.get_analog(ANALOG_LEFT_Y);
        int right = masterController.get_analog(ANALOG_RIGHT_Y);

        // Brake controls for drifting???
        int brakeLeft = masterController.get_digital(DIGITAL_L2);
        int brakeRight = masterController.get_digital(DIGITAL_R2);

        // Intake manual controls
        int intakeUp = masterController.get_digital(DIGITAL_L1);
        int intakeDown = masterController.get_digital(DIGITAL_R1);

        // Forklift 1 manual controls
        int forklift1Up = masterController.get_digital(DIGITAL_UP);
        int forklift1Down = masterController.get_digital(DIGITAL_DOWN);

        // Forklift 2 manual controls
        int forklift2Up = masterController.get_digital(DIGITAL_X);
        int forklift2Down = masterController.get_digital(DIGITAL_B);

        // Hold drivetrain macro
        bool holdDrivetrainInput = masterController.get_digital_new_press(DIGITAL_Y);

        // Intake macro
        int intakeMacroCW = masterController.get_digital_new_press(DIGITAL_LEFT);
        int intakeMacroCCW = masterController.get_digital_new_press(DIGITAL_RIGHT);
        
        int forkliftSpeed = 100;
        int intakeSpeed = 80; // Also can be 40

        double threshold = 0;

        if (maintenanceToggle) { 
            // while the special button is pressed for any motor, and while left joystick
            // is moved, motor speed is toggled
            // only holding the button makes that motor run at current saved speed
            
            // left key for forklift 1 motor
            if (getDigitalPress(DIGITAL_LEFT)) { 
                forklift1.setPower(getLeftJoystick());
            }
            
            // right key for forklift 2 motor
            if (getDigitalPress(DIGITAL_RIGHT)) { 
                forklift2.setPower(getLeftJoystick());
            }

            // up key for intake motor
            if (getDigitalPress(DIGITAL_UP)) { 
                intake.setPower(getLeftJoystick());
            }

            // L1 for tleft motor
            if (getDigitalPress(DIGITAL_L1)) { 
                tLeft.move(getLeftJoystick());
            }

            // R1 for tRight motor
            if (getDigitalPress(DIGITAL_R2)) {
                tRight.move(getLeftJoystick());
            }

            // L2 for bLeft motor
            if (getDigitalPress(DIGITAL_L2)) { 
                bLeft.move(getLeftJoystick());
            }

            // R2 for bRight motor
            if (getDigitalPress(DIGITAL_R2)) {
                bRight.move(getLeftJoystick());
            }
        } else {
            // Pass joystick values to drivetrain
            // driveTrain->arcade(forward, yaw, 0);
            // driveTrain->arcadeWithBrakes(forward, yaw, brakeLeft, brakeRight, 0);
            driveTrain->tank(joystickCubicDrive(left), joystickCubicDrive(right), threshold);

            // Intake macro handler
            if (macroToggle != 0 && intakeMacroCCW) {
                // Turn off macro if already on
                macroToggle = 0;
            } else {
                // State does not match the button pressed, enable the respective macro
                if (intakeMacroCW) {
                    // Change macro state to clockwise
                    macroToggle = 1;
                } else if (intakeMacroCCW) {
                    // Change macro state to counter clockwise
                    macroToggle = -1;
                }
            }

            if (holdDrivetrainInput) {
                drivetrainHold = !drivetrainHold;
            }

            if (drivetrainHold) {
                tLeft.set_brake_mode(MOTOR_BRAKE_BRAKE);
                tRight.set_brake_mode(MOTOR_BRAKE_BRAKE);
                bLeft.set_brake_mode(MOTOR_BRAKE_BRAKE);
                bRight.set_brake_mode(MOTOR_BRAKE_BRAKE);
            } else {
                tLeft.set_brake_mode(MOTOR_BRAKE_COAST);
                tRight.set_brake_mode(MOTOR_BRAKE_COAST);
                bLeft.set_brake_mode(MOTOR_BRAKE_COAST);
                bRight.set_brake_mode(MOTOR_BRAKE_COAST);
            }



            // Apply macro control to intake system
            switch (macroToggle) {
                case 1: {
                    startIntakeSmoothMove(false, false);
                    break;
                }
                case -1: {
                    startIntakeSmoothMove(true, true);
                    break;
                }
                case 0: {
                    // Operator control
                    intake.control();

                    if (intakeUp) {
                        intake.setPower(intakeSpeed);
                    } else if (intakeDown) {
                        intake.setPower(-intakeSpeed);
                    } else {
                        intake.setPower(0);
                    }
                    break;
                }
                default: {
                    break;
                }
            }
            intake.update();

            // Forklift 1 control
            forklift1.control();

            if (forklift1Up) {
                forklift1.setPower(forkliftSpeed);
            } else if (forklift1Down) {
                forklift1.setPower(-forkliftSpeed);
            } else {
                forklift1.setPower(0);
            }

            // Run update funcs on sysmans
            forklift1.update();

            // Forklift 2 control
            forklift2.control();

            if (forklift2Up) {
                forklift2.setPower(forkliftSpeed);
            } else if (forklift2Down) {
                forklift2.setPower(-forkliftSpeed);
            } else {
                forklift2.setPower(0);
            }

            // Run update funcs on sysmans
            forklift2.update();
            
        }
        pros::delay(10);
    } // close while    
}