# Super's exact, safety-checked polyline from the current path projection to
# the rolling local goal.  The two counters bind this reference to one global
# path transaction and one local-goal update.
std_msgs/Header header
uint64 global_path_id
uint64 local_goal_seq
geometry_msgs/Point local_goal
geometry_msgs/Point[] points
# Safety-checked continuation of the same global polyline after local_goal.
# Normal EGO planning uses only points; recovery uses continuation_points to
# choose a rejoin target without extrapolating a single yaw through a corner.
geometry_msgs/Point[] continuation_points
float64 arc_length
float64 minimum_clearance
