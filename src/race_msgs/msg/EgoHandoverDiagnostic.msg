# Read-only evidence for one EGO trajectory handover. This message never
# participates in control or safety decisions.
uint8 SOURCE_BRIDGE=1
uint8 EVENT_CANDIDATE_COMMITTED=1
uint8 EVENT_SWITCH_REJECTED=2
uint8 EVENT_TRANSACTION_EXHAUSTED=3

std_msgs/Header header
uint8 source
uint8 event
uint64 local_goal_seq
int64 active_trajectory_id
int64 candidate_trajectory_id

# Candidate timing as observed by the bridge.
float64 candidate_start_age_sec
float64 candidate_elapsed_sec
float64 active_remaining_sec

# The two states compared by the Bridge at this exact callback time.
geometry_msgs/Point active_position
geometry_msgs/Vector3 active_velocity
geometry_msgs/Vector3 active_acceleration
geometry_msgs/Point candidate_position
geometry_msgs/Vector3 candidate_velocity
geometry_msgs/Vector3 candidate_acceleration
float64 position_error
float64 velocity_error
float64 acceleration_error
string reason
