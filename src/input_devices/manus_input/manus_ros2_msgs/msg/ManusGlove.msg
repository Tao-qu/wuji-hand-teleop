# Stamp at receipt of a new SDK raw-skeleton frame, not the ROS publish timer.
std_msgs/Header header
# Per-glove sequence advances only on a new SDK skeleton frame.
uint64 sequence
int32 glove_id
string side
int32 raw_node_count
ManusRawNode[] raw_nodes
int32 ergonomics_count
ManusErgonomics[] ergonomics
geometry_msgs/Quaternion raw_sensor_orientation
int32 raw_sensor_count
geometry_msgs/Pose[] raw_sensor
