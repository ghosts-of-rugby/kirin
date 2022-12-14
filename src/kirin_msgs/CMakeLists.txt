cmake_minimum_required(VERSION 3.5)
project(kirin_msgs)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joint.msg"
  "msg/AirState.msg"
  "msg/ColorLED.msg"
  "msg/HandState.msg"
  "msg/HandPosition.msg"
  "msg/Motor.msg"
  "msg/MotorStateVector.msg"
  "msg/MoveMode.msg"
  "msg/ZAutoState.msg"
  "srv/StartRapidHand.srv"
  "srv/StartZAutoMovement.srv"
  "srv/StartPlanarAutoMovement.srv"
  "srv/SetAirState.srv"
  "srv/SetColorLED.srv"
  "srv/SetHandState.srv"
  "srv/ToggleHandState.srv"
  "srv/SetTarget.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
