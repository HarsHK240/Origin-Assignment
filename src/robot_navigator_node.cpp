#include "assignment/robot_navigator_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stdexcept>

RobotNavigatorNode::RobotNavigatorNode() : Node("robot_navigator") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // cmd_vel publisher
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(  
        "odom", 10, std::bind(&RobotNavigatorNode::odom_callback, this, std::placeholders::_1)); //odom subscriber

    waypoints_ = {{0.0, 0.0}, {2.0, 1.0}, {4.0, -1.0}, {5.0, 2.0}};
    
    try {
        auto smoothed_path = planner_.smooth_path(waypoints_);
        trajectory_ = planner_.generate_trajectory(smoothed_path);
        RCLCPP_INFO(this->get_logger(), "Navigator Initialized. Spline path generated.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize trajectory: %s", e.what());
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&RobotNavigatorNode::control_loop, this));
}

void RobotNavigatorNode::set_emergency_stop(bool stop_active) {
    e_stop_active_ = stop_active;
    if (stop_active) {
        RCLCPP_WARN(this->get_logger(), "MANUAL OVERRIDE: Robot stopped.");
    } else {
        RCLCPP_INFO(this->get_logger(), "MANUAL OVERRIDE: Resuming trajectory.");
    }
}

void RobotNavigatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_state_.theta = yaw;

    current_state_.v = msg->twist.twist.linear.x;
    current_state_.w = msg->twist.twist.angular.z;
}

void RobotNavigatorNode::control_loop() {
    try {
        if (e_stop_active_) {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            return;
        }

        if (trajectory_.empty()) return;
        auto cmd = tracker_.compute_velocity_command(current_state_, trajectory_);
        cmd_vel_pub_->publish(cmd);
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Control Loop Error: %s", e.what());
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        timer_->cancel(); 
    }
}