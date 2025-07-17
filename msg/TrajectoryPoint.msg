# Trajectory point for mixed distance/position trajectory execution
# Supports both relative distance movements and absolute position movements

# Command type - determines which fields are used
string command_type     # 'distance' or 'position'

# For distance commands (relative movement)
float64 left_distance    # meters (relative distance to move)
float64 right_distance   # meters (relative distance to move)

# For position commands (absolute positioning - servo mode)
float64 left_position    # radians (absolute wheel position)
float64 right_position   # radians (absolute wheel position)
float64 deceleration    # m/s² (deceleration rate for position commands)

# Common parameters for both command types
float64 speed           # m/s (max speed during movement)
float64 acceleration    # m/s² (acceleration rate)
float64 duration        # seconds (optional timing constraint, 0 = no constraint)