#include "assignment/trajectory_tracker.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

TrajectoryTracker::TrajectoryTracker() {}

// Calculating [v_min, v_max, w_min, w_max] based on kinematics and current speed
std::vector<double> TrajectoryTracker::calculate_dynamic_window(const RobotState& state) {
    return {
        std::max(min_speed_, state.v - max_accel_ * dt_),
        std::min(max_speed_, state.v + max_accel_ * dt_),
        std::max(-max_yaw_rate_, state.w - max_delta_yaw_rate_ * dt_),
        std::min(max_yaw_rate_, state.w + max_delta_yaw_rate_ * dt_)
    };
}

// Simulating where the robot will go if it holds (v, w) for predict_time_
std::vector<RobotState> TrajectoryTracker::predict_trajectory(RobotState state, double v, double w) {
    std::vector<RobotState> traj;
    double time = 0.0;
    while (time <= predict_time_) {
        state.theta += w * dt_;
        state.x += v * std::cos(state.theta) * dt_;
        state.y += v * std::sin(state.theta) * dt_;
        state.v = v;
        state.w = w;
        traj.push_back(state);
        time += dt_;
    }
    return traj;
}

double TrajectoryTracker::calculate_heading_cost(const std::vector<RobotState>& traj, const TrajectoryPoint& target) {
    RobotState final_state = traj.back();
    double dx = target.x - final_state.x;
    double dy = target.y - final_state.y;
    double target_angle = std::atan2(dy, dx);
    
    double error_angle = target_angle - final_state.theta;
    while (error_angle > M_PI) error_angle -= 2.0 * M_PI;
    while (error_angle < -M_PI) error_angle += 2.0 * M_PI;
    
    return std::abs(error_angle);
}

// EXTRA CREDIT HOOK: Returns 0 for now. Pass in a LaserScan or Costmap here to penalize trajectories near obstacles. 
double TrajectoryTracker::calculate_clearance_cost(const std::vector<RobotState>& traj) {
    (void)traj; // Suppress unused warning
    return 0.0; 
}

geometry_msgs::msg::Twist TrajectoryTracker::compute_velocity_command(
    const RobotState& current_state,
    const std::vector<TrajectoryPoint>& trajectory) {
    
    geometry_msgs::msg::Twist best_cmd;
    // Safety check: Halt if no trajectory exists or we have finished
    if (trajectory.empty() || current_target_idx_ >= trajectory.size()) return best_cmd;

    TrajectoryPoint target = trajectory[current_target_idx_];
    
    // Check if we reached the current waypoint (Accepting radius = 0.2m) 
    double dist_to_target = std::hypot(target.x - current_state.x, target.y - current_state.y);
    if (dist_to_target < 0.2) {
        current_target_idx_++;
        if (current_target_idx_ >= trajectory.size()) return best_cmd; // Finished
        target = trajectory[current_target_idx_];
    }

    std::vector<double> dw = calculate_dynamic_window(current_state);
    
    double min_cost = std::numeric_limits<double>::max();
    double best_v = 0.0;
    double best_w = 0.0;

    // Evaluate all possible (v, w) pairs within the dynamic window
    for (double v = dw[0]; v <= dw[1]; v += v_resolution_) {
        for (double w = dw[2]; w <= dw[3]; w += w_resolution_) {
            
            std::vector<RobotState> predicted_traj = predict_trajectory(current_state, v, w);
            
            double heading_cost = heading_weight_ * calculate_heading_cost(predicted_traj, target);
            double clearance_cost = clearance_weight_ * calculate_clearance_cost(predicted_traj);
            double velocity_cost = velocity_weight_ * (max_speed_ - v); 
            
            double total_cost = heading_cost + clearance_cost + velocity_cost;
            
            if (total_cost < min_cost) {
                min_cost = total_cost;
                best_v = v;
                best_w = w;
            }
        }
    }

    best_cmd.linear.x = best_v;
    best_cmd.angular.z = best_w;
    return best_cmd;
}