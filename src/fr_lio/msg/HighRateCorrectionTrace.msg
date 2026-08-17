# One entry per accepted high-rate reset from a LiDAR posterior.
# header.stamp is the LiDAR anchor time; propagated_timestamp identifies the
# IMU state that was replayed when the correction was committed.
std_msgs/Header header

float64 propagated_timestamp
float64 smoothing_elapsed_before
float64 smoothing_duration
geometry_msgs/Vector3 propagated_velocity
geometry_msgs/Vector3 anchor_posterior_velocity
geometry_msgs/Vector3 posterior_velocity
geometry_msgs/Vector3 output_velocity_before
geometry_msgs/Vector3 output_velocity_after
geometry_msgs/Vector3 correction_offset_velocity
bool smoothing_enabled
