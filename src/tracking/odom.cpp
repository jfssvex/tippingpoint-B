#include "tracking.h"
#include "main.h"
#include "globals.h"
#include "chassis.h"
#include <math.h>

// Encoder deltas
float lDelta; // Delta of distance travelled by left tracking wheel
float rDelta; // Delta of distance travelled by right tracking wheel
float bDelta; // Delta of distance travelled by back tracking wheel

// Real world distances
float lDist; // lDelta in inches
float rDist; // rDelta in inches
float bDist; // bDelta in inches
float aDelta; // Delta of angle in radians

// Previous encoder values
float lLast; // Last value of left tracking wheel
float rLast; // Last value of right tracking wheel
float bLast; // Last value of back tracking wheel

// Constants and macros
const float lrOffset = WHEELBASE / 2.0f; // Offset of the left / right tracking wheel from the center in terms of x axis
const float bOffset = -BACK_WHEEL_OFFSET; // Offset of the back tracking wheel from the center in terms of y axis (negative because its in the back)

// Conversion calculations
#define DRIVE_DEGREE_TO_INCH (M_PI * DRIVE_WHEEL_DIAMETER / 360) 
#define TRACKING_WHEEL_DEGREE_TO_INCH (M_PI * TRACKING_WHEEL_DIAMETER / 360)

// Actual tracking function that runs in BG
void tracking(void* parameter) {
    // Assuming that there is one IMU
    // Also, I don't know if this works

    Vector2 globalPos(0, 0);

    // Tracking loop
    while (true) {
        Vector2 localPos;

        // Get sensor values
        pros::c::imu_accel_s_t accelerationData = myImu.get_accel();

        printf("Delta X: %f, Delta Y: %f", accelerationData.x, accelerationData.y);

        // Finally, update the global position (does not work well)
        if (accelerationData.x < INFINITY || accelerationData.y < INFINITY) {
            globalPos = Vector2(
                globalPos.getX() + round(accelerationData.x * 10) / 10,
                globalPos.getY() + round(accelerationData.y * 10) / 10
            );
        }
        
        // Update tracking data
        trackingData.update(globalPos.getX(), globalPos.getY(), myImu.get_heading());

        // Debug print (can't use display so just throw to serial)
        printf("X: %f, Y: %f, A: %f\n", 
                trackingData.getPos().getX(), 
                trackingData.getPos().getY(), 
                radToDeg(trackingData.getHeading()));
        
        pros::delay(10); // Max of 10ms
    }
}