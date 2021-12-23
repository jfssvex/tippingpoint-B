#include "tracking.h"
#include "main.h"
#include <atomic>

Vector2::Vector2(double x, double y) {
    std::atomic_init(&this->x, x);
    std::atomic_init(&this->y, y);
    // this->x = x;
    // this->y = y;
}

Vector2::Vector2() {
    std::atomic_init(&this->x, 0);
    std::atomic_init(&this->y, 0);
    // this->x = 0;
    // this->y = 0;
}

double Vector2::getMagnitude() {
    // Use pythagorean theorem
    return sqrt(std::atomic_load(&x)*std::atomic_load(&x) + std::atomic_load(&y)*std::atomic_load(&y));
}

double Vector2::getAngle() {
    return atan2(std::atomic_load(&y), std::atomic_load(&x));
}

Vector2 Vector2::normalize() {
    // Divide x and y by magnitude to retain direction
    return Vector2(this->x / this->getMagnitude(), this->y / this->getMagnitude());
}

Vector2 operator+(const Vector2 &v1, const Vector2 &v2) {
    return Vector2(std::atomic_load(&v1.x) + std::atomic_load(&v2.x), std::atomic_load(&v1.y) + std::atomic_load(&v2.y));
}

Vector2 operator-(const Vector2 &v1, const Vector2 &v2) {
    return Vector2(std::atomic_load(&v1.x) - std::atomic_load(&v2.x), std::atomic_load(&v1.y) - std::atomic_load(&v2.y));
}

Vector2 operator*(const Vector2 &v1, const double scalar) {
    return Vector2(std::atomic_load(&v1.x) * scalar, std::atomic_load(&v1.y) * scalar);
}