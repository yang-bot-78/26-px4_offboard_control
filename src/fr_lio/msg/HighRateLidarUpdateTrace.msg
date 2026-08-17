# One entry per accepted LiDAR update. header.stamp is the LiDAR anchor time.
std_msgs/Header header

geometry_msgs/Vector3 pre_velocity
geometry_msgs/Vector3 posterior_velocity
geometry_msgs/Vector3 delta_velocity

float64 residual_rms
float64 delta_position_z
float64 delta_accel_bias_z
uint32 effective_points
