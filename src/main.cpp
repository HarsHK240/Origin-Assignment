#include <rclcpp/rclcpp.hpp>
#include "assignment/robot_navigator_node.hpp"
#include <memory>
#include <thread>
#include <iostream>
#include <string>

void terminal_listener(std::shared_ptr<RobotNavigatorNode> node) {
    std::string input;
    std::cout << "  TYPE 'q' + ENTER TO STOP THE ROBOT" << std::endl;
    std::cout << "  TYPE 'r' + ENTER TO RESUME" << std::endl;

    while (rclcpp::ok()) {
        std::cin >> input;
        
        if (input == "q") {
            // Trigger the flag to publish zero velocities
            node->set_emergency_stop(true);
        } 
        else if (input == "r") {
            // Un-trigger the flag to resume DWA tracking
            node->set_emergency_stop(false);
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotNavigatorNode>();
    std::thread listener_thread(terminal_listener, node);
    listener_thread.detach(); 

    rclcpp::spin(node);
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    
    return 0;
}