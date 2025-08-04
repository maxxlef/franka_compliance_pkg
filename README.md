# Arm Workspace

This workspace contains three main ROS 2 packages dedicated to Franka Emika arm.

---

## Packages Overview

- [arm_pkg](https://github.com/maxxlef/arm_ws/tree/main/src/arm_pkg): Handles compliance control, force sensing, and smooth motion planning. Includes nodes for force bridging and wrench services.
- [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers): Provides Cartesian control for the Franka arm, enabling precise manipulation and interaction with the environment.
- [franka_arm_ros2](https://github.com/tstoyanov/franka_arm_ros2): ROS 2 interface for the Franka Emika arm, facilitating communication and control.

---

## Usage

Each package contains its own README with detailed descriptions and usage instructions.  
Visit the links above to explore each package individually.

---

## Installation

1. Install ROS 2 Humble on your system.

2. Clone this repository into your ROS 2 workspace:
   ```bash
   git clone <this_repository_url>
   ```

3. Install cartesian compliance controller, see [cartesian_compliance_controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
Delete `cartesian_controller_simulation` and `cartesian_controller_tests` before building if you don't need them (otherwise see [Cartesian Controllers Simulation](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/cartesian_controller_simulation/README.md)).

4. Install Franka ROS 2 packages, see [franka_ros2](https://github.com/tstoyanov/franka_arm_ros2)

5. Build the workspace:
   ```bash
   colcon build
    ```

6. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---
## License

This project is licensed under the MIT License.  
See the [LICENSE](./LICENSE) file for details.

- [`cartesian_compliance_controller`](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/cartesian_controller_simulation/)
  
  ➤ License: Apache 2.0

- [`franka_ros2`](https://github.com/tstoyanov/franka_arm_ros2)
  
  ➤ License: BSD-2-Clause
  

--- 

Happy grasping!
