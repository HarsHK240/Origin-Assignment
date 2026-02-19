#ifndef TRAJECTORY_ASSIGNMENT_PATH_PLANNER_HPP_
#define TRAJECTORY_ASSIGNMENT_PATH_PLANNER_HPP_

#include "assignment/types.hpp"
#include <vector>

class PathPlanner {
public:
    std::vector<Point2D> smooth_path(const std::vector<Point2D>& waypoints);
    std::vector<TrajectoryPoint> generate_trajectory(const std::vector<Point2D>& path);
};

#endif // TRAJECTORY_ASSIGNMENT_PATH_PLANNER_HPP_
