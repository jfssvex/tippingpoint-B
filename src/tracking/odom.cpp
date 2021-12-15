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

float  halfA = 0;

// Constants and macros
const float lrOffset = WHEELBASE / 2.0f; // Offset of the left / right tracking wheel from the center in terms of x axis
// const float bOffset = -BACK_WHEEL_OFFSET; // Offset of the back tracking wheel from the center in terms of y axis (negative because its in the back)
const float bOffset = 0; // Offset of the back tracking wheel, 0 because there is none

bool printTracking = true;

// Conversion calculations
#define DRIVE_DEGREE_TO_INCH (M_PI * DRIVE_WHEEL_DIAMETER / 360) 
#define TRACKING_WHEEL_DEGREE_TO_INCH (M_PI * TRACKING_WHEEL_DIAMETER / 360)

// Actual tracking function that runs in BG
void tracking(void* param) {

	// Initialize variables
	lLast = 0;
	rLast = 0;
	bLast = 0;
	float x = trackingData.getPos().getX();
	float y = trackingData.getPos().getY();
	float left = 0;
	float right = 0;
	float lateral = 0;
	float angle = 0;

	uint32_t printTime = pros::millis();

	// Start tracking loop
	while(1) {
		float localX, localY = 0;

		// Get encoder data
		float leftEncoder = bLeft.get_position();
		float rightEncoder = 0;
		float backEncoder = 0;

		// Calculate deltas
		lDelta = leftEncoder - lLast;
		rDelta = rightEncoder - rLast;
		bDelta = backEncoder - bLast;
		lDist = lDelta * DRIVE_DEGREE_TO_INCH;
		rDist = rDelta * DRIVE_DEGREE_TO_INCH;
		bDist = bDelta * DRIVE_DEGREE_TO_INCH;

		// Store readings for next deltas
		lLast = leftEncoder;
		rLast =	rightEncoder;
		bLast = backEncoder;

		// Increment persistent variables
		left += lDist;
		right += rDist;
		lateral += bDist;

		// Calculate arc angle
		float holdAngle = angle;
		angle = (right - left)/(lrOffset * 2.0f);
		aDelta = angle - holdAngle;

		// If theres an arc
		if(aDelta) {
			localY = ((rDist / aDelta - lrOffset) * sin(aDelta/2.0f)) * 2.0f;

			localX = ((bDist / aDelta - BACK_WHEEL_OFFSET) * sin(aDelta/2.0f)) * 2.0f;
		}
		// If no arc
		else {
			halfA = 0;
			aDelta = 0;
			localY = (rDist+lDist)/2;
			localX = bDist;
		}

		float p = -(halfA + holdAngle); // The global ending angle of the robot
		float cosP = cos(p);
		float sinP = sin(p);

		// Update the global position
		float dY = (localY * cosP) - (localX * sinP);
		float dX = (localY * sinP) + (localX * cosP);

		trackingData.update(trackingData.getPos().getX() + dX, trackingData.getPos().getY() +dY, trackingData.getHeading() + aDelta);

		// Print debug
		if(pros::millis() - printTime > 50) {
			printf("X: %f, Y: %f, A: %f\n", trackingData.getPos().getX(), trackingData.getPos().getY(), radToDeg(trackingData.getHeading()));
			printTime = pros::millis();
			// printf("X: %f, Y: %f", left, right);
			// printf("A: %f", lateral);
		}

		pros::delay(5);
	}
}