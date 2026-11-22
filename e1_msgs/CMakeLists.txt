cmake_minimum_required(VERSION 3.8)
project(e1_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlightState.msg" "msg/AttitudeTarget.msg" "msg/ManualControl.msg" "msg/MotorCommand.msg"
  "msg/PositionTarget.msg" "msg/SwarmStatus.msg"
  "srv/SetMode.srv" "srv/SetParam.srv" "srv/Takeoff.srv" "srv/Land.srv" "srv/GoTo.srv" "srv/Console.srv"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
