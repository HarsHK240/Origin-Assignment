#ifndef ASSIGNMENT_ROBOT_NAVIGATOR_NODE_HPP_
#define ASSIGNMENT_ROBOT_NAVIGATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "assignment/path_planner.hpp"
#include "assignment/trajectory_tracker.hpp"
#include "assignment/types.hpp"
#include <vector>
#include <atomic> // NEW: For thread-safe flags

class RobotNavigatorNode : public rclcpp::Node {
public:
    RobotNavigatorNode();
    
    // NEW: Public method to allow external threads to pause/resume the robot
    void set_emergency_stop(bool stop_active);

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PathPlanner planner_;
    TrajectoryTracker tracker_;

    std::vector<Point2D> waypoints_;
    std::vector<TrajectoryPoint> trajectory_;
    RobotState current_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // NEW: Thread-safe flag to pause the control loop
    std::atomic<bool> e_stop_active_{false}; 
};

#endif // ASSIGNMENT_ROBOT_NAVIGATOR_NODE_HPP_