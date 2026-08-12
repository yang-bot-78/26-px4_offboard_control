# Planner-to-controller contract. All values use header.frame_id.
std_msgs/Header header
geometry_msgs/Point position

# Optional path-tangent velocity feed-forward.
bool velocity_valid
geometry_msgs/Vector3 velocity

# Heading and optional heading-rate feed-forward in radians and radians/second.
float64 yaw
bool yaw_rate_valid
float64 yaw_rate
