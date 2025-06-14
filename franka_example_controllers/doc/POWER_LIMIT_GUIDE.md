# Avoiding Power Limit Violations in Franka Controllers

This document describes how to avoid the `power_limit_violation` error that can occur when using Franka robots with the `joint_impedance_example_controller`.

## Background

The Franka Emika Panda robot has safety mechanisms that detect excessive power consumption. When the mechanical power exceeds a certain threshold (typically around 50-80W), the robot will enter a reflex mode and abort the motion with an error message:

```
[ERROR] libfranka: Move command aborted: motion aborted by reflex! ["power_limit_violation"]
```

This error commonly occurs when:
1. The controller applies excessive torques
2. The controller gains are too high
3. The controller tries to make rapid movements
4. There is a sudden change in commanded torque

## Solutions Implemented

The improved `joint_impedance_example_controller` includes several mechanisms to prevent power limit violations:

### 1. Smooth Controller Startup

The controller now has a gradual ramp-up phase with configurable duration (default: 1.0 seconds):
- Initial pure gravity compensation with no motion control (first 300ms)
- Quadratic gain scaling for smoother torque application
- Gradual increase of control gains during startup

### 2. Power Monitoring and Limiting

The controller actively monitors mechanical power:
- Computes power as: P = τ * ω (torque × angular velocity)
- Scales down commanded torques when approaching power limits
- Applies more conservative torque rate limits during startup

### 3. Improved Torque Rate Limiting

The controller implements more sophisticated torque rate limiting:
- Reduces maximum torque change rate during startup
- Enforces absolute torque limits per joint
- Applies additional safeguards during the first control cycle

## Configuration Parameters

To adjust the controller's behavior, modify these parameters in `joint_impedance_safe.yaml`:

```yaml
# Power and safety limits
power_limit: 50.0      # Maximum mechanical power allowed (W)
tau_limit: 87.0        # Maximum joint torque (Nm)
startup_duration: 1.0  # Duration for controller startup (s)

# Reduced gains for safety
joint_kp: [200.0, 200.0, 200.0, 200.0, 150.0, 300.0, 100.0]  # Position gains
joint_kd: [15.0, 15.0, 15.0, 15.0, 6.0, 12.0, 4.0]           # Damping gains
```

## Usage

To use the safer controller configuration:

```bash
# Launch the controller with safer defaults
roslaunch franka_example_controllers joint_impedance_safe.launch robot_ip:=<robot-ip>
```

## Troubleshooting

If you still encounter power limit violations:

1. **Reduce gains further**: Lower the `joint_kp` and `joint_kd` values in the configuration
2. **Increase startup duration**: Set `startup_duration` to 1.5 or 2.0 seconds
3. **Lower the movement speed**: Reduce `max_delta_q` values
4. **Check for external forces**: Ensure nothing is obstructing the robot's movement

## Additional Resources

- [Franka Control Interface Documentation](https://frankaemika.github.io/docs/)
- [Franka Error Handling Guide](https://frankaemika.github.io/docs/troubleshooting.html)
