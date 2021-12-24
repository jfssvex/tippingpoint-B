#include "systems/systemManager.h"
#include "globals.h"

void updateSystemManagers(void* param) {
    // Enable all systems
    intake.enable();
    forklift1.enable();
    forklift2.enable();
    
    while (true) {
        // Update all system managers

        // Doesn't work because it's a pure virtual, though i'll try to find a way
        // since this is 10x more efficient and scalable
        /**
        for (SystemManager& sysMan : systemManagers) {
            sysMan.update();
        }
        */

        forklift1.update();
        forklift2.update();
        intake.update();

        pros::delay(20);
    }
}