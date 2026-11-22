# Position/velocity/acceleration setpoint for the ground position controller (e1_control).
std_msgs/Header header          # frame_id = world
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw
