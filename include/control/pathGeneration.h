#include "main.h"
#include "tracking.h"

// In inches
#define POINT_INJECT_SPACING 6

#ifndef _PATH_GEN_H_
#define _PATH_GEN_H_

struct Waypoint {
    double x, y; // Inches
    double heading; // Radians

    double curvature; // Not sure what type this should be
    double distance; // distance (as the crow flies) of this point from beginning of path
};

class Path {
    private:
        std::vector<Vector2> desiredPoints;
        std::vector<Waypoint> waypoints;

        /**
         * @brief Inject points in-between the path-defining points
         * @param spacing The desired distance between each point after injection
         */
        void injectPoints(double spacing);

        /**
         * @brief Optimize the path by smoothing them using gradient descent.
         * This algorithm was borrowed from FRC Team 2168 and can be seen here:
         * https://github.com/KHEngineering/SmoothPathPlanner/blob/11059aa2ec314ba20b364aeea3c968aca2672b49/src/usfirst/frc/team2168/robot/FalconPathPlanner.java#L214 
         * @param weightData How much to weigh the waypoint data
         * @param weightSmooth How much smoothing should occur
         * @param tolerance The tolerance of change
         */
        void smoothPath(double weightData, double weightSmooth, double tolerance);
    
    public:
        Path();
        Path(std::vector<Vector2> points);

        void addPoint(Vector2 point);
        void preparePath();
        
};

#endif