# Internal recording transport; does not command hardware.
int64 frame_index
int64 timestamp_ns
float64[54] action_joint_position
float64[14] action_ee_pose
float64[54] state_joint_position
float64[14] state_ee_pose
float64[54] state_joint_velocity
# action left/right arm, action left/right hand, action left/right EE,
# state left/right arm, state left/right hand; all ROS source stamps in ns.
int64[10] source_timestamps_ns
