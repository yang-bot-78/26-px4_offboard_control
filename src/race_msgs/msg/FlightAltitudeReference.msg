std_msgs/Header header

# Monotonic identifier assigned by the Offboard authority for each takeoff.
uint64 flight_id
bool valid

# All configured vertical limits are heights above the locked takeoff ground.
float64 target_agl_m
float64 min_agl_m
float64 max_agl_m

# PX4 local position uses NED (positive down).
float64 ground_z_local_ned
float64 target_z_local_ned

# Navigation and EGO use map ENU (positive up).
float64 ground_z_map
float64 target_z_map
