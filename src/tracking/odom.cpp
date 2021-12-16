#include "tracking.h"
#include "main.h"
#include "globals.h"
#include "chassis.h"
#include <math.h>

// Encoder deltas
float lDelta = 0; // Delta of distance travelled by left tracking wheel
float rDelta = 0; // Delta of distance travelled by right tracking wheel
float bDelta = 0; // Delta of distance travelled by back tracking wheel

// Real world distances
float lDist = 0; // lDelta in inches
float rDist = 0; // rDelta in inches
float bDist = 0; // bDelta in inches
float aDelta = 0; // Delta of angle in radians

// Previous encoder values
float lLast = 0; // Last value of left tracking wheel
float rLast = 0; // Last value of right tracking wheel
float bLast = 0; // Last value of back tracking wheel

// Constants and macros
const float lrOffset = WHEELBASE / 2.0f; // Offset of the left / right tracking wheel from the center in terms of x axis
// const float bOffset = -BACK_WHEEL_OFFSET; // Offset of the back tracking wheel from the center in terms of y axis (negative because its in the back)
const float bOffset = 0; // Offset of the back tracking wheel, 0 because there is none

// Conversion calculations
#define DRIVE_DEGREE_TO_INCH (M_PI * DRIVE_WHEEL_DIAMETER / 360) 
#define TRACKING_WHEEL_DEGREE_TO_INCH (M_PI * TRACKING_WHEEL_DIAMETER / 360)

bool printTracking = true;

// Actual tracking function that runs in BG
void tracking(void* parameter) {
    // Assuming that there are 3 encoders
    // Also, I don't know if this works

    // Initialize variables
    lLast = 0; // Last encoder value of left
    rLast = 0; // Last encoder value of right
    bLast = 0; // Last encoder value of back

    Vector2 globalPos(0, 0);

    float left = 0; // Total distance travelled by left tracking wheel
    float right = 0; // Total distance travelled by right tracking wheel
    float lateral = 0; // Total distance travelled laterally (measured from back tracking wheel)
    float angle = 0; // Current arc angle

    uint32_t printTime = pros::millis();

    /*
    // Reset all motor positions
    for (pros::Motor* tmp : driveTrain->allMotors) {
        tmp->tare_position();
    }
    */

    // Reset encoders to 0 before starting
    // lEnc.reset();
    // rEnc.reset();
    // bEnc.reset();

    // Tracking loop
    while (true) {
        Vector2 localPos;

        // Get encoder data, directly fron wheels because no tracking wheels yet
        float lEncVal = (tLeft.get_position() + bLeft.get_position());
        float rEncVal = (tRight.get_position() + bRight.get_position());
        float bEncVal = 0;

        // Calculate delta values
        lDelta = lEncVal - lLast;
        rDelta = rEncVal - rLast;
        bDelta = bEncVal - bLast;

        // Calculate IRL distances from deltas
        lDist = lDelta * DRIVE_DEGREE_TO_INCH;
        rDist = rDelta * DRIVE_DEGREE_TO_INCH;
        bDist = bDelta * DRIVE_DEGREE_TO_INCH;

        // Update last values for next iter since we don't need to use last values for this iteration
        lLast = lEncVal;
        rLast = rEncVal;
        bLast = bEncVal;

        // Update total distance vars
        left += lDist;
        right += rDist;
        lateral += bDist;

        // Calculate new absolute orientation
        float prevAngle = angle; // Previous angle, used for delta
        // angle = (right - left) / (lrOffset * 2.0f);
        angle = myImu.get_rotation(); // Using IMU, maybe not the best idea but meh

        // Get angle delta
        aDelta = angle - prevAngle;

        // Calculate using different formulas based on if orientation change
        float avgLRDelta = (lDist + rDist) / 2; // Average of delta distance travelled by left and right wheels
        if (aDelta == 0.0f) {
            // Set the local positions to the distances travelled since the angle didn't change
            localPos = Vector2(bDist, avgLRDelta);
        } else {
            // Use the angle to calculate the local position since angle did change
            localPos = Vector2(
                2 * sin(aDelta / 2) * (bDist / aDelta - bOffset),
                2 * sin(aDelta / 2) * (rDist / aDelta - lrOffset)
            );
        }

        // Calculate the average orientation
        // If any issues arise, try changing aDelta to aDelta/2
        float avgAngle = -(prevAngle + (aDelta))/2;

        // Calculate global offset https://www.mathsisfun.com/polar-cartesian-coordinates.html
        float globalOffsetX = cos(avgAngle); // cos(θ) = x 
        float globalOffsetY = sin(avgAngle); // sin(θ) = y 

        // Finally, update the global position
        globalPos = Vector2(
            trackingData.getPos().getX() + (localPos.getY() * globalOffsetY) + (localPos.getX() * globalOffsetX),
            trackingData.getPos().getY() + (localPos.getY() * globalOffsetX) - (localPos.getX() * globalOffsetY)
        );

        // Update tracking data
        // trackingData.update(globalPos.getX(), globalPos.getY(), degToRad(myImu.get_rotation()));
        trackingData.update(globalPos, trackingData.getHeading() + aDelta);

        // Debug print
        if (pros::millis() - printTime > 75 && printTracking) {
            // Only print every 75ms to reduce lag
            printf("X: %f, Y: %f, A: %f\n", 
                    trackingData.getPos().getX(), 
                    trackingData.getPos().getY(), 
                    radToDeg(trackingData.getHeading()));
            printTime = pros::millis();
        }
        
        pros::delay(5); // Max of 10ms
    }
}