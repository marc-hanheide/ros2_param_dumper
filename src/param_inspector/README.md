# ROS2 Parameter Inspector

This package provides tools for dumping and inspecting runtime ROS2 parameters across all nodes in a ROS2 system.

## Description

The `param_inspector` package allows you to collect and inspect parameters from all running ROS2 nodes in your system. It generates both a YAML file and a formatted Markdown report containing parameter values, types, and descriptions.

## Installation

Note, the repository itself can serve as a ROS2 workspace, or it can be cloned into other workspaces

```bash
# Clone the repository into your ROS2 workspace
cd <your_workspace>/src
git clone <repository_url>

# Build the package
cd ..
colcon build
```

## Usage

After installation, you can run the parameter inspector using:

```bash
source install/setup.bash

ros2 run param_inspector param_report
```

This will:
1. Collect parameters from all running ROS2 nodes
2. Generate a formatted Markdown report to stdout
3. Save all parameters to `ros2_parameters.yaml`

## Output Format

The tool generates two types of output:

1. **Markdown Report**: A human-readable table format showing:
    - Node names
    - Parameter names
    - Parameter values
    - Parameter types
    - Parameter descriptions

2. **YAML File**: A machine-readable dump of all parameters in `ros2_parameters.yaml`

## Dependencies

- rclpy
- rclpy_message_converter
- rcl_interfaces

## License

This package is licensed under the Apache-2.0 license.

## Author

Maintained by Marc Hanheide (marc@hanheide.net)