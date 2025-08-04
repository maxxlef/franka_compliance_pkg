# Franka_compliance_pkg
This ROS 2 (Humble) package provides C++ nodes for control a Franka Emika arm thanks to the cartesian compliance controller. It contains several nodes for compliance control thanks to a bridge between franka_arm_ros2 and cartesian_controllers,  a smooth motion planning, and a force sensing service for grasp detection. 

You can test the arm with an interactive marker in RViz.

---

## Packages Overview

- [franka_compliance_pkg](https://github.com/maxxlef/franka_compliance_pkg#overview-franka_compliance_pkg): Handles compliance control, force sensing, and smooth motion planning. Includes nodes for force bridging and wrench services.
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
   git clone https://github.com/maxxlef/franka_compliance_ws.git
   ```

3. Install cartesian compliance controller, see [cartesian_compliance_controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
Delete `cartesian_controller_simulation` and `cartesian_controller_tests` before building if you don't need them (otherwise see [Cartesian Controllers Simulation](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/cartesian_controller_simulation/README.md)).

4. Install Franka ROS 2 packages, see [franka_ros2](https://github.com/tstoyanov/franka_arm_ros2)

5. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the workspace:
   ```bash
   colcon build
    ```

6. Source the workspace:
   ```bash
   source install/setup.bash
   ```
7. After switching the arm to FCI mode launch the nodes:
   ```bash
   ros2 launch franka_compliance_pkg compliance_control.launch.py
   ```

---



## Overview franka_compliance_pkg

### Nodes

- **`force_bridge_node`**  
  Publishes wrench data from the Franka API as `geometry_msgs/WrenchStamped`, intended for the `/cartesian_compliance_controller/ft_sensor_wrench` input topic. It compensates for gravity.

- **`wrench_service_node`**  
  Offers a service for grasp detection, thanks to a simple thresholding on the measured force.

- **`interactive_marker_node`**
  Publishes a 3D interactive marker in RViz to control a target pose for the arm. Useful for manual testing or debugging.

- **`motion_smoother_node`**  
  Smooths the target pose trajectory using interpolation, ensuring the arm moves smoothly to the desired position.

### Parameters

This package expects runtime parameters such as:
- The mass and centre of mass of the tool attached to the end of the arm
- Force thresholds for the grasp detection service
- Publish rate and step size for the motion smoother (this is how we can limit the speed)
- Input and output topics for the motion smoother (the input topic is a PoseStamped that defines the target pose)

These parameters should be defined in a centralized configuration file. See [`/config/params.yaml`](https://github.com/maxxlef/arm_ws/blob/main/src/arm_pkg/config/params.yaml) for details.
You can easily adjust them there without modifying codes.

--- 

## Dependencies

This package depends on the following ROS 2 components:

- ament_cmake
- rclcpp
- interactive_markers
- visualization_msgs
- geometry_msgs
- tf2_ros
- tf2
- tf2_geometry_msgs
- franka_msgs
- std_srvs

Make sure all dependencies are available in your workspace before building. 
You can install them using the following command from the root of your workspace:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## License

This project is licensed under the MIT License.  
For details see the [LICENSE](./LICENSE) and:

- [`cartesian_compliance_controller`](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/cartesian_controller_simulation/)
  
  ➤ License: Apache 2.0

- [`franka_ros2`](https://github.com/tstoyanov/franka_arm_ros2)
  
  ➤ License: BSD-2-Clause
  

--- 

Happy smooth control!
