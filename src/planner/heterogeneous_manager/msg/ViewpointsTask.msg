# ViewpointsTask.msg
int32 drone_id

float64 stamp
float64 recent_attempt_time

# viewpoint pose
int32[] orders
geometry_msgs/Point[] points
float64[] pitchs
float64[] yaws