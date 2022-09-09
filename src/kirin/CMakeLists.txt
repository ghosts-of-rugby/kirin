cmake_minimum_required(VERSION 3.5)
project(kirin)

set(CMAKE_CXX_STANDARD 20)

# ddt motor directory
add_subdirectory(lib/ddt-motor)

# magic enum
add_subdirectory(lib/magic_enum)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(kirin_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB src_files src/*.cpp)
file(GLOB gencode_files gencode/*.cpp)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} main/${target}.cpp ${src_files} ${gencode_files})
  ament_target_dependencies(${target} 
    "rclcpp"
    "tf2"
    "tf2_eigen"
    "std_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "kirin_msgs"
  )
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
  target_link_libraries(${target}
    ddtmotor
    magic_enum
    ${Eigen3_LIBRARIES}
  )
endfunction()

custom_executable(kirin_main_executor)
custom_executable(jagariko_marker_publisher)
custom_executable(jsk_visualize_node)
custom_executable(test_motor_controller)
custom_executable(test_hand_tool_manager)

include_directories(
  include
  gencode
  /usr/local/MATLAB/R2020a/extern/include
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../../lib/magic_enum/include
)


install(DIRECTORY
    launch
    resources
    urdf
    rviz
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()

