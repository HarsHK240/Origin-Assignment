
# Path Smoothing and Trajectory Control in 2D Space

This repository implements a path smoothing algorithm and a trajectory tracking controller for a differential drive robot navigating through a series of 2D waypoints. It bridges discrete global planning with continuous local execution to ensure smooth and safe motion.

##  Project Structure

```sh
└── Origin-Assignment/
    ├── CMakeLists.txt
    ├── include
    │   └── assignment
    │       ├── path_planner.hpp
    │       ├── robot_navigator_node.hpp
    │       ├── trajectory_tracker.hpp
    │       └── types.hpp
    ├── launch
    │   └── tracker_launch.py
    ├── package.xml
    ├── src
    │   ├── main.cpp
    │   ├── path_planner.cpp
    │   ├── robot_navigator_node.cpp
    │   └── trajectory_tracker.cpp
    └── test
        └── test_path_planner.cpp
```

## 2.1 Setup and Execution Instructions

**Prerequisites:**

* Ubuntu 22.04 with ROS 2 Humble installed.
* Standard ROS 2 Turtlebot3 simulation packages (`sudo apt install ros-humble-turtlebot3-gazebo`).
* GTest framework for unit testing.

**Build Instructions:**

1. Clone this package into the `src` directory of your ROS 2 workspace (e.g., `origin_ws/src/assignment`).
2. Navigate to the workspace root and resolve dependencies:
```bash
cd ~/origin_ws
rosdep install -i --from-path src --rosdistro humble -y

```


3. Build the package:
```bash
colcon build --packages-select assignment

```



**Execution:**
To utilize the background terminal listener (which allows you to pause/resume the robot via the command line), the node must be run directly rather than through a launch file.

1. **Terminal 1 (Start Simulation):**
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py

```


2. **Terminal 2 (Run Node):**
```bash
cd ~/origin_ws
source install/setup.bash
ros2 run assignment trajectory_node

```


*Note: Type `q` + Enter in Terminal 2 to trigger the emergency stop, and `r` + Enter to resume tracking.*

**Running Automated Tests:**

```bash
colcon test --packages-select assignment
colcon test-result --all

```

---

## 2.2 Design Choices, Algorithms, and Architectural Decisions

The codebase is strictly modular, decoupling the complex control mathematics from the ROS 2 middleware to maximize testability and reusability.

* 
**Path Smoothing (Piecewise Cubic Bezier Curve):** Instead of standard linear interpolation or basic splines, a Piecewise Cubic Bezier curve algorithm was implemented to generate a smooth, continuous path. This guarantees  continuity (smooth tangent transitions) across all segments. A tunable tension factor is used to calculate control points, allowing the robot to take sweeping, natural arcs rather than jerky, in-place rotations.


* 
**Trajectory Generation (Trapezoidal Velocity Profile):** To generate a time-parameterized trajectory , the geometric path is augmented with a trapezoidal velocity profile. By applying real-world kinematic constraints (max velocity and max acceleration), the algorithm calculates exact timestamps based on physical acceleration, cruising, and deceleration phases.


* 
**Controller (Dynamic Window Approach - DWA):** Rather than a naive PID loop, the trajectory tracking controller utilizes DWA. DWA samples a "dynamic window" of reachable linear and angular velocities  based on the robot's current speed and acceleration limits. It predicts the future trajectory for each velocity pair and selects the command that minimizes a cost function (heading alignment, speed, and clearance). This ensures the differential drive robot only receives physically executable commands.



---

## 2.3 Extending to a Real Robot

Due to the decoupled, object-oriented architecture, deploying this to a physical differential drive platform—such as a 4-wheeled robot utilizing a Jetson/Arduino hardware bridge—requires zero changes to the core `PathPlanner` or `TrajectoryTracker` C++ logic.

The extension process is isolated entirely within the `RobotNavigatorNode`:

1. **Sensor Input:** The simulated Gazebo `/odom` subscription would be replaced by the physical robot's odometry topic, heavily relying on the Arduino's wheel encoder data (and potentially an IMU filtered via an Extended Kalman Filter) to populate the `RobotState`.
2. **Motor Output:** The `/cmd_vel` output remains identical. The node would publish standard `Twist` messages to a serial-bridge node. The Arduino would subscribe to these commands, parse the linear and angular velocities, and feed them into its low-level hardware PID loops to physically drive the DC motors.
3. **Parameter Tuning:** The kinematic limits inside `trajectory_tracker.hpp` (e.g., `max_speed_`, `max_accel_`) would be adjusted from the Turtlebot3 defaults to match the physical capabilities of the specific hardware chassis.

---

## 2.4 AI Tools Used

Modern AI tools were actively utilized to support the development workflow and establish best practices. Google Gemini was primarily used to:

* Scaffold standard ROS 2 C++ boilerplate (CMakeLists, package dependencies).
* Generate the `GTest` framework for robust unit testing.
* Assist in refactoring the complex mathematical implementations of the Bezier curve matrices and ensuring safe floating-point operations in the control loop.
* Writing README.md 

---

## 2.5 Extra Credit: Obstacle Avoidance Extension

The decision to implement the Dynamic Window Approach (DWA) perfectly primes this system for obstacle avoidance. DWA naturally evaluates a "clearance" score for possible movements.

To extend this codebase for obstacle avoidance:

1. **Sensor Integration:** Add a ROS 2 subscription inside the `RobotNavigatorNode` to listen to a 2D LiDAR topic (`sensor_msgs/msg/LaserScan`) or a local costmap.
2. **State Updating:** Pass the latest scan data into the `compute_velocity_command` loop.
3. **Cost Calculation:** Implement the currently empty `calculate_clearance_cost` function in the `TrajectoryTracker`. For every predicted trajectory in the dynamic window, the algorithm would check the distance between the simulated robot positions and the coordinates of the LiDAR hits.
4. **Penalization:** If any predicted  trajectory brings the robot within a defined safety radius of an obstacle, the clearance cost returns `infinity`, immediately discarding that velocity option and forcing the robot to steer around the obstruction.



