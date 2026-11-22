# Attitude + collective thrust command (-> e1/<id>/cmd/attitude). Switches the drone to AUTO.
# Must be streamed at >= 10 Hz; after ZN_CMD_TIMEOUT (0.5 s) without commands the drone descends.
std_msgs/Header header
geometry_msgs/Quaternion orientation   # desired body FLU -> world
geometry_msgs/Vector3 body_rate        # feed-forward body rates (rad/s), used if use_body_rate
bool use_body_rate
float32 thrust                         # collective duty [0,1]; 0 disarms
