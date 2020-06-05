cmake_minimum_required(VERSION 3.5)
project(robot_controllers_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerState.msg"
  "msg/DiffDriveLimiterParams.msg"
  "action/QueryControllerStates.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
