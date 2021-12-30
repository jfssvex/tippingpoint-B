#include "main.h"
#include "chassis.h"
#include "systemManager.h"
#include "control/PID.h"

//to do:
// add inheritance from system managers
// add PID for upPos, downPos
class Arm: public SystemManager {
    public:
        //States
        static const uint8_t DISABLED_STATE = 0x00;
        static const uint8_t RESET_STATE = 0x01;
        static const uint8_t UP_STATE = 0x02; // 
        static const uint8_t DOWN_STATE = 0x03; // loading rings
        static const uint8_t OPERATOR_OVERRIDE = 0x20;

        //Arm motor
        pros::Motor armMotor = pros::Motor(ARM_PORT, pros::E_MOTOR_GEARSET_18, true);

        //Constructor
        Arm(uint8_t defaultState, double downPos, double upPos, PIDInfo constants);

        //Destructor
        ~Arm();

        //Moves arm
        void setPower(int armPower);

        void raise();

        void lower();

        //Allows for an operator to control the system
        void control();

        void update() override;

        bool changeState(uint8_t newState);

        void fullReset() override;
        

    protected:
        int armManualPower = 0;

        double downPos;
        double upPos;

        //PID
        PIDInfo constants;
        PIDController *pidController;
};