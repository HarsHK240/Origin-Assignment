#include "assignment/path_planner.hpp"
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <stdexcept>

double get_distance(const Point2D& p1, const Point2D& p2) {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

// Cubic Bezier Curve Equation
// B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
Point2D calculate_bezier_point(const Point2D& p0, const Point2D& p1, 
                               const Point2D& p2, const Point2D& p3, double t) {
    // Variable declared in such a way to reduce computation    
    double u = 1.0 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;

    Point2D p;
    p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    return p;
}


std::vector<Point2D> PathPlanner::smooth_path(const std::vector<Point2D>& waypoints) {
    std::vector<Point2D> smoothed;
    if (waypoints.empty()) return smoothed;
    if (waypoints.size() < 2) return waypoints; 

    int points_per_segment = 20; 

    // Iterating through each segment between waypoints
    for (std::size_t i = 0; i < waypoints.size() - 1; ++i) {
        Point2D p0 = waypoints[i];
        Point2D p3 = waypoints[i+1];
        
        // Calculating intermediate control points (P1 and P2) to ensure smoothness.
        Point2D p1, p2;
        double tension = 0.25; 

        if (i == 0) {
            // First segment boundary handling
            p1 = {p0.x + (p3.x - p0.x) * tension, p0.y + (p3.y - p0.y) * tension};
        } else {
            Point2D p_prev = waypoints[i-1];
            p1 = {p0.x + (p3.x - p_prev.x) * tension, p0.y + (p3.y - p_prev.y) * tension};
        }

        if (i == waypoints.size() - 2) {
            // Last segment boundary handling
            p2 = {p3.x - (p3.x - p0.x) * tension, p3.y - (p3.y - p0.y) * tension};
        } else {
            Point2D p_next = waypoints[i+2];
            p2 = {p3.x - (p_next.x - p0.x) * tension, p3.y - (p_next.y - p0.y) * tension};
        }

        // Generating the points along this Bezier segment
        for (int j = 0; j < points_per_segment; ++j) {
            double t = static_cast<double>(j) / points_per_segment;
            smoothed.push_back(calculate_bezier_point(p0, p1, p2, p3, t));
        }
    }
    
    // Ensure the very last waypoint is explicitly included at the end of the path
    smoothed.push_back(waypoints.back());
    return smoothed;
}


std::vector<TrajectoryPoint> PathPlanner::generate_trajectory(const std::vector<Point2D>& path) {
    std::vector<TrajectoryPoint> traj;
    if (path.empty()) return traj;

    const double max_v = 0.22;  
    const double max_a = 0.5;   
    
    // Calculating total path length and cumulative distance to each point
    std::vector<double> cum_dist(path.size(), 0.0);
    double total_distance = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        total_distance += get_distance(path[i-1], path[i]);
        cum_dist[i] = total_distance;
    }

    // Edge case
    if (total_distance < 1e-6) {
        traj.push_back({path[0].x, path[0].y, 0.0, 0.0}); 
        return traj;
    }

    // Distance required to accelerate from 0 to max_v
    double accel_dist = (max_v * max_v) / (2.0 * max_a);
    bool reaches_max_v = (total_distance >= 2.0 * accel_dist);
    double current_time = 0.0;
    
    for (std::size_t i = 0; i < path.size(); ++i) {
        double s = cum_dist[i];
        double v_target = 0.0;

        if (reaches_max_v) {
            if (s < accel_dist) v_target = std::sqrt(2.0 * max_a * s); //Accelerating
            else if (s > total_distance - accel_dist) v_target = std::sqrt(2.0 * max_a * (total_distance - s)); //Decelerating
            else v_target = max_v; //Cruising
        } 
        else {
            if (s < total_distance / 2.0) v_target = std::sqrt(2.0 * max_a * s); //Triangular profile when Path is too short to cruise
            else v_target = std::sqrt(2.0 * max_a * (total_distance - s));
        }

        v_target = std::max(v_target, 0.01); 

        //Calculating Timestamp
        if (i > 0) {
            double ds = get_distance(path[i-1], path[i]);
            double prev_v = traj.back().v;
            double v_avg = (prev_v + v_target) / 2.0;
            current_time += (ds / v_avg);
        }

        traj.push_back({path[i].x, path[i].y, current_time, v_target});
    }

    // Final velocity = 0
    traj.back().v = 0.0;
    return traj;
}