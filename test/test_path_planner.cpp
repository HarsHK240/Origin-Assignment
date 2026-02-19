#include <gtest/gtest.h>
#include "assignment/path_planner.hpp"
#include <vector>
#include <cmath>

// Test Case 1: Error Handling - Empty Input
TEST(PathPlannerTest, EmptyWaypoints) {
    PathPlanner planner;
    std::vector<Point2D> empty_input;
    
    auto result = planner.smooth_path(empty_input);
    
    // We expect the planner to safely return an empty path without crashing
    EXPECT_TRUE(result.empty());
}

// Test Case 2: Error Handling - Single Waypoint
TEST(PathPlannerTest, SingleWaypoint) {
    PathPlanner planner;
    std::vector<Point2D> single_input = {{1.0, 1.0}};
    
    auto result = planner.smooth_path(single_input);
    
    // A single point cannot form a curve. It should return the point safely.
    EXPECT_EQ(result.size(), 1);
    EXPECT_DOUBLE_EQ(result[0].x, 1.0);
}

// Test Case 3: Bezier Interpolation Count and Endpoints
TEST(PathPlannerTest, BezierSegmentCountAndEndpoints) {
    PathPlanner planner;
    std::vector<Point2D> waypoints = {{0.0, 0.0}, {10.0, 0.0}}; // 1 segment
    
    auto result = planner.smooth_path(waypoints);
    
    // The Bezier logic creates 20 points per segment, plus the final explicitly added waypoint.
    EXPECT_EQ(result.size(), 21);
    
    // Check if the start and end points match exactly
    EXPECT_DOUBLE_EQ(result.front().x, 0.0);
    EXPECT_DOUBLE_EQ(result.front().y, 0.0);
    EXPECT_DOUBLE_EQ(result.back().x, 10.0);
    EXPECT_DOUBLE_EQ(result.back().y, 0.0);
}

// Test Case 4: Multiple Segments Continuity
TEST(PathPlannerTest, MultiSegmentPath) {
    PathPlanner planner;
    // 3 waypoints = 2 segments = (20 * 2) + 1 = 41 points
    std::vector<Point2D> waypoints = {{0.0, 0.0}, {5.0, 5.0}, {10.0, 0.0}};
    
    auto result = planner.smooth_path(waypoints);
    
    EXPECT_EQ(result.size(), 41);
}

// Test Case 5: Error Handling - Empty Path Trajectory
TEST(TrajectoryTest, EmptyPath) {
    PathPlanner planner;
    std::vector<Point2D> empty_path;
    
    auto traj = planner.generate_trajectory(empty_path);
    EXPECT_TRUE(traj.empty());
}

// Test Case 6: Edge Case - Zero Length Path
TEST(TrajectoryTest, ZeroLengthPath) {
    PathPlanner planner;
    std::vector<Point2D> zero_length = {{0.0, 0.0}, {0.0, 0.0}};
    
    auto traj = planner.generate_trajectory(zero_length);
    
    // Should return a safe resting state
    ASSERT_EQ(traj.size(), 1);
    EXPECT_DOUBLE_EQ(traj[0].v, 0.0);
    EXPECT_DOUBLE_EQ(traj[0].t, 0.0);
}

// Test Case 7: Trapezoidal Kinematics Validation
TEST(TrajectoryTest, ValidKinematics) {
    PathPlanner planner;
    // Generate a long, straight path so it has room to accelerate, cruise, and brake
    std::vector<Point2D> path;
    for(int i = 0; i <= 50; i++) {
        path.push_back({static_cast<double>(i), 0.0}); 
    }
    
    auto traj = planner.generate_trajectory(path);
    
    ASSERT_FALSE(traj.empty());
    
    // 1. The robot must end at an absolute stop
    EXPECT_DOUBLE_EQ(traj.back().v, 0.0);
    
    // 2. Timestamps must strictly increase
    for(size_t i = 1; i < traj.size(); ++i) {
        EXPECT_GT(traj[i].t, traj[i-1].t);
    }
    
    // 3. Ensure velocity never exceeds the 0.22 m/s limit set in the code
    for(const auto& point : traj) {
        EXPECT_LE(point.v, 0.22 + 1e-6); // Added small epsilon for floating point math
    }
}

// Standard GTest main function
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}