#ifndef ASSIGNMENT_TYPES_HPP_
#define ASSIGNMENT_TYPES_HPP_

struct Point2D { double x; double y; };
struct TrajectoryPoint { double x; double y; double t; double v; };

// NEW: Required for DWA to know current velocities
struct RobotState { 
    double x; 
    double y; 
    double theta; 
    double v; // Linear velocity
    double w; // Angular velocity
};

#endif // ASSIGNMENT_TYPES_HPP_