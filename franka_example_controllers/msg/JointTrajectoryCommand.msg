# Joint trajectory command message for deoxys-compatible trajectory interpolation
# This message allows sending multiple trajectory points with timestamps

Header header

# Array of trajectory points
JointTrajectoryPoint[] points

# Trajectory configuration
float64 time_fraction        # Time scaling factor (default: 1.0)
bool is_delta               # Whether positions are relative to current (default: false)
float64[] max_delta_q       # Maximum position change per joint per control cycle
