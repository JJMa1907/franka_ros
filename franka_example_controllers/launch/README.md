# Joint Impedance Controller Launch Files

This directory contains several launch files for the enhanced joint impedance controller with deoxys compatibility:

## Launch Files

### 1. joint_impedance_example_controller.launch (Original)
- **Purpose**: Original launch file, now enhanced with deoxys-compatible parameters
- **Features**: 
  - Full robot initialization with franka_control
  - RViz visualization  
  - Enhanced configuration with deoxys parameters
- **Usage**: `roslaunch franka_example_controllers joint_impedance_example_controller.launch robot:=panda`

### 2. joint_impedance_deoxys_compatible.launch (Identical)
- **Purpose**: Explicitly named deoxys-compatible version (identical to original)
- **Features**: Same as original - included for clarity
- **Usage**: `roslaunch franka_example_controllers joint_impedance_deoxys_compatible.launch robot:=panda`

### 3. joint_impedance_test.launch (Testing)
- **Purpose**: Lightweight testing version
- **Features**: 
  - Optional RViz (default: false)
  - Faster startup for testing
- **Usage**: 
  ```bash
  # Without RViz
  roslaunch franka_example_controllers joint_impedance_test.launch robot:=panda
  
  # With RViz  
  roslaunch franka_example_controllers joint_impedance_test.launch robot:=panda rviz:=true
  ```

## Configuration

All launch files use the same enhanced configuration from `config/franka_example_controllers.yaml`, which now includes:

- ✅ Deoxys-compatible joint gains (joint_kp, joint_kd)
- ✅ Trajectory interpolation parameters (max_delta_q, time_fraction)
- ✅ State estimation parameters (alpha_q, alpha_dq)
- ✅ Delta command support (is_delta)
- ✅ Joint limit protection (joint_limit_margin)

## Quick Start

1. **For normal usage** (with visualization):
   ```bash
   roslaunch franka_example_controllers joint_impedance_example_controller.launch robot:=panda
   ```

2. **For testing** (without visualization):
   ```bash
   roslaunch franka_example_controllers joint_impedance_test.launch robot:=panda
   ```

3. **Send commands**:
   ```bash
   # Joint position command
   rostopic pub /joint_impedance_example_controller/joint_command std_msgs/Float64MultiArray "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"
   
   # Enable delta mode for incremental commands
   rosparam set /joint_impedance_example_controller/is_delta true
   rostopic pub /joint_impedance_example_controller/joint_command std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
   ```

For detailed implementation information, see `DEOXYS_COMPATIBILITY_GUIDE.md`.
