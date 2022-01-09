#include "control/pathGeneration.h"

Path::Path() {

}

Path::Path(std::vector<Vector2> points) {
    this->desiredPoints = points;
}

void Path::addPoint(Vector2 point) {
    desiredPoints.push_back(point);
}

void Path::preparePath() {
    injectPoints(POINT_INJECT_SPACING);
    smoothPath(0.25, 0.75, 0.001);
}

void Path::injectPoints(double spacing) {
    waypoints.clear();
    
    for (int i = 0; i < desiredPoints.size() - 1; i++) {
        std::pair<Vector2, Vector2> lineSegment(desiredPoints[i], desiredPoints[i + 1]);

        Vector2 displacement = lineSegment.second - lineSegment.first;
        int maxFittablePoints = ceil(displacement.getMagnitude() / spacing);
        Vector2 pointInject = displacement.normalize() * spacing;

        for (int x = 0; x < maxFittablePoints; x++) {
            Vector2 newPoint = lineSegment.first + pointInject * i; 

            Waypoint waypoint;
            waypoint.x = newPoint.getX();
            waypoint.y = newPoint.getY();

            waypoints.push_back(waypoint);
        }

        Waypoint lastWaypoint;
        lastWaypoint.x = desiredPoints[desiredPoints.size() - 1].getX();
        lastWaypoint.y = desiredPoints[desiredPoints.size() - 1].getY();
        waypoints.push_back(lastWaypoint);
    }
}

void Path::smoothPath(double weightData, double weightSmooth, double tolerance) {
    double change = tolerance;
    std::vector<Waypoint> waypointsOrig(waypoints);

    while (change >= tolerance) {
        change = 0;

        for (int i = 1; i < waypoints.size() - 1; i++) {
            waypoints[i].x = 
                weightData * (waypointsOrig[i].x - waypoints[i].x) + 
                weightSmooth * (waypoints[i - 1].x + waypoints[i + 1].x - (2 * waypoints[i].x));
            
            waypoints[i].y = 
                weightData * (waypointsOrig[i].y - waypoints[i].y) + 
                weightSmooth * (waypoints[i - 1].y + waypoints[i + 1].y - (2 * waypoints[i].y));

            change += abs(waypointsOrig[i].x - waypoints[i].x) + abs(waypointsOrig[i].y - waypoints[i].y);
        }
    }
}