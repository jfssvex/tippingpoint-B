#include "tracking.h"
#include <math.h>
#include <atomic>

TrackingData::TrackingData(double x, double y, double h) {
	this->pos = Vector2(x, y);
    std::atomic_store(&this->heading, fmod(h, 2 * M_PI));
}

double TrackingData::getHeading() {
    return std::atomic_load(&this->heading);
}

Vector2 TrackingData::getPos() {
    return this->pos;
}

Vector2 TrackingData::getForward() {
    return toGlobalCoordinates(Vector2(0, 1));
}

void TrackingData::update(double newX, double newY, double newH) {
    this->pos = Vector2(newX, newY);
    
    if(!this->suspendModulus) {
		// Make sure heading is between -2pi and +2pi
		std::atomic_store(&this->heading, fmod(newH, 2 * M_PI));
	}
	else {
		std::atomic_store(&this->heading, newH);
	}
}

void TrackingData::update(Vector2 newPos, double newH) {
    this->pos = newPos;

    if(!this->suspendModulus) {
		// Make sure heading is between -2pi and +2pi
		std::atomic_store(&this->heading, fmod(newH, 2 * M_PI));
	}
	else {
		std::atomic_store(&this->heading, newH);
	}
}

void TrackingData::setAngleModulusSuspend(bool suspend) {
    this->suspendModulus = suspend;
}