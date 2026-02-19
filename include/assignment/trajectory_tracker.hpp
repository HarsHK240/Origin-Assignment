#ifndef ASSIGNMENT_TRAJECTORY_TRACKER_HPP_
#define ASSIGNMENT_TRAJECTORY_TRACKER_HPP_

#include "assignment/types.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

class TrajectoryTracker {
public:
    TrajectoryTracker();
    
    // Updated to accept full RobotState
    geometry_msgs::msg::Twist compute_velocity_command(
        const RobotState& current_state,
        const std::vector<TrajectoryPoint>& trajectory);

private:
    size_t current_target_idx_ = 0;

    // DWA Kinematic Constraints (Tuned for typical differential drive like Turtlebot/Zebra)
    double max_speed_ = 0.62;       // m/s
    double min_speed_ = 0.0;        // m/s (no reversing for simplicity)
    double max_yaw_rate_ = 2.84;    // rad/s
    double max_accel_ = 1.0;        // m/s^2
    double max_delta_yaw_rate_ = 3.2; // rad/s^2
    
    // DWA Simulation Parameters
    double dt_ = 0.05;               // Time step for prediction
    double predict_time_ = 1.5;     // How far ahead to simulate (seconds)
    double v_resolution_ = 0.02;    // Velocity sampling resolution
    double w_resolution_ = 0.01;     // Yaw rate sampling resolution

    // DWA Cost Function Weights
    double heading_weight_ = 2.0;
    double clearance_weight_ = 1.0; // Ready for Extra Credit 
    double velocity_weight_ = 0.2;

    // DWA Helper Functions
    std::vector<double> calculate_dynamic_window(const RobotState& state);
    std::vector<RobotState> predict_trajectory(RobotState init_state, double v, double w);
    double calculate_heading_cost(const std::vector<RobotState>& predicted_traj, const TrajectoryPoint& target);
    double calculate_clearance_cost(const std::vector<RobotState>& predicted_traj);
};

#endif // ASSIGNMENT_TRAJECTORY_TRACKER_HPP_