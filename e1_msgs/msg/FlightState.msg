# Telemetry of one E1 Nano (from e1/<id>/state, 50 Hz)
std_msgs/Header header          # frame_id = <ns>/base_link
uint8 drone_id
uint8 RAW=0
uint8 ACRO=1
uint8 STAB=2
uint8 ALTHOLD=3
uint8 AUTO=4
uint8 mode
bool armed
bool landed
bool low_battery
bool cmd_timeout                # AUTO commands stopped -> firmware is descending
bool failsafe
geometry_msgs/Quaternion attitude       # onboard estimate, body FLU -> world
geometry_msgs/Vector3 angular_velocity  # rad/s body
geometry_msgs/Vector3 linear_acceleration  # m/s^2 body (specific force)
float32[4] motors               # duty [0,1], order: rear_left, rear_right, front_right, front_left
float32 thrust                  # collective thrust target (duty)
float32 battery_voltage
uint16 seq
