# Dual Impedance Controller

A unified ROS controller that integrates both Cartesian and Joint impedance control modes for the Franka Emika Panda robot, allowing dynamic switching between control modes during operation.

## Features

- **Dual Mode Operation**: Switch between Cartesian and Joint impedance control modes
- **Runtime Mode Switching**: Change control modes without stopping the controller
- **Unified Interface**: Single controller combining functionality from both original controllers
- **Topic-based Control**: Simple Bool topic for mode switching
- **Service Interface**: ROS service for programmatic mode switching
- **Parameter Compatibility**: Supports all parameters from both original controllers

## Control Modes

### Cartesian Impedance Mode (`true`)
- Controls end-effector pose in Cartesian space
- Configurable stiffness and damping in 6 DOF
- Force/torque estimation and publishing
- Nullspace control for redundancy resolution
- Dynamic reconfigure support

### Joint Impedance Mode (`false`)
- Controls individual joint positions
- Trajectory interpolation support
- External command mode for programmatic control
- Delta and absolute position modes
- Joint limit enforcement

## Topics

### Control Topics
- `/impedance_mode` (std_msgs/Bool): Mode command (true=cartesian, false=joint)
- `/current_impedance_mode` (std_msgs/String): Current mode status

### Cartesian Mode Topics
- `/equilibrium_pose` (geometry_msgs/PoseStamped): Target pose
- `/equilibrium_configuration` (std_msgs/Float32MultiArray): Nullspace configuration
- `/stiffness` (std_msgs/Float32MultiArray): Stiffness parameters
- `/cartesian_pose` (geometry_msgs/PoseStamped): Current pose
- `/force_torque_ext` (geometry_msgs/WrenchStamped): Estimated external forces

### Joint Mode Topics
- `/joint_command` (std_msgs/Float64MultiArray): Joint position commands
- `/torque_comparison` (franka_example_controllers/JointTorqueComparison): Torque data

## Services

- `/switch_impedance_mode` (std_srvs/SetBool): Switch control mode

## Parameters

### Robot Configuration
```yaml
arm_id: "panda"
joint_names: [panda_joint1, panda_joint2, ..., panda_joint7]
```

### Cartesian Impedance Parameters
```yaml
cartesian_stiffness:
  translational_stiffness_X: 1000.0  # N/m
  translational_stiffness_Y: 1000.0
  translational_stiffness_Z: 1000.0
  rotational_stiffness_X: 30.0       # Nm/rad
  rotational_stiffness_Y: 30.0
  rotational_stiffness_Z: 30.0

nullspace_stiffness: 10.0
```

### Joint Impedance Parameters
```yaml
joint_kp: [300.0, 300.0, 300.0, 300.0, 225.0, 450.0, 150.0]  # Nm/rad
joint_kd: [20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0]          # Nm⋅s/rad
max_delta_q: [0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06]     # rad/cycle
```

### Control Parameters
```yaml
use_external_command: true   # Enable external joint commands
is_delta: false             # false=absolute, true=relative positions
coriolis_factor: 1.0        # Coriolis compensation factor
publish_rate: 30.0          # Diagnostic publishing rate
```

## Usage

### Launch the Controller

```bash
# Start with Cartesian mode (default)
roslaunch franka_example_controllers dual_impedance_controller.launch

# Start with Joint mode
roslaunch franka_example_controllers dual_impedance_controller.launch start_mode:=joint

# Specify robot IP
roslaunch franka_example_controllers dual_impedance_controller.launch robot_ip:=172.16.0.2
```

### Switch Modes

#### Using Topics
```bash
# Switch to Cartesian mode
rostopic pub /impedance_mode std_msgs/Bool "data: true"

# Switch to Joint mode
rostopic pub /impedance_mode std_msgs/Bool "data: false"
```

#### Using Services
```bash
# Switch to Cartesian mode
rosservice call /switch_impedance_mode "data: true"

# Switch to Joint mode
rosservice call /switch_impedance_mode "data: false"
```

### Send Commands

#### Cartesian Commands
```bash
# Send target pose
rostopic pub /equilibrium_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'panda_link0'
pose:
  position: {x: 0.4, y: 0.0, z: 0.5}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

# Send stiffness values [trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, nullspace]
rostopic pub /stiffness std_msgs/Float32MultiArray "data: [1000, 1000, 1000, 30, 30, 30, 10]"
```

#### Joint Commands
```bash
# Send joint positions (7 values in radians)
rostopic pub /joint_command std_msgs/Float64MultiArray "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"
```

### Run Demo
```bash
# Run automated demo
rosrun franka_example_controllers dual_impedance_demo.py
```

## Configuration Files

- `config/dual_impedance_controller.yaml`: Main configuration
- `launch/dual_impedance_controller.launch`: Launch file
- `scripts/dual_mode_switcher.py`: Mode switching service
- `scripts/dual_impedance_demo.py`: Demonstration script

## Safety Features

- Torque rate limiting to prevent sudden movements
- Joint limit enforcement with soft boundaries
- Power and torque limit monitoring
- Smooth mode transitions
- State estimation with filtering

## Implementation Details

### Mode Switching
- Modes can be switched at runtime without stopping the controller
- Current state is preserved during transitions
- Both modes share the same torque output interface
- Mode status is published for monitoring

### Control Architecture
```
User Commands → Mode Router → [Cartesian Controller | Joint Controller] → Torque Commands → Robot
```

### Thread Safety
- Real-time safe mode switching
- Atomic state updates
- No memory allocation in control loop

## Troubleshooting

### Common Issues

1. **Controller fails to start**
   - Check robot connection and FCI activation
   - Verify parameter file paths
   - Ensure proper joint names configuration

2. **Mode switching not working**
   - Check topic connections: `rostopic list | grep impedance`
   - Verify controller is running: `rosnode list`
   - Check parameter loading: `rosparam list`

3. **Commands not executed**
   - Verify correct mode is active
   - Check topic names and message types
   - Monitor controller logs for errors

### Monitoring Commands
```bash
# Check current mode
rostopic echo /current_impedance_mode

# Monitor controller status
rostopic echo /cartesian_pose               # Cartesian mode
rostopic echo /torque_comparison            # Joint mode

# Check parameters
rosparam get /dual_impedance_controller/
```

## Integration with Existing Code

This controller is designed to be a drop-in replacement for either:
- `CartesianImpedanceExampleController`
- `JointImpedanceExampleController`

Simply change the controller name in your launch files and add mode switching capability.

## Contributing

When modifying the controller:
1. Maintain real-time safety in the control loop
2. Test both control modes thoroughly
3. Update configuration documentation
4. Add appropriate error handling

## License

This controller follows the same Apache-2.0 license as the franka_example_controllers package.
