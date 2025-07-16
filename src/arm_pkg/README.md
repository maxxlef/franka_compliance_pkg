# arm_compliance_control

This ROS 2 (Humble) package provides C++ nodes for control a Franka Emika arm thanks to the cartesian compliance controller through the `ros2_control` framework.

## Overview

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
- Mass and inertia of the arm
- Force thresholds
- Publish rate and step size for the motion smoother
- Input and output topics for the motion smoother

These parameters should be defined in a centralized configuration file. See [`/config/params.yaml`](TODO) for details.
You can easily adjust them there without modifying codes.


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

## License

