cmake_minimum_required(VERSION 3.5)
project(kirin)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(kirin_msgs REQUIRED)

file(GLOB src_files src/*.cpp)
file(GLOB gencode_files gencode/*.cpp)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} main/${target}.cpp ${src_files} ${gencode_files})
  ament_target_dependencies(${target} 
    "rclcpp"
    "tf2_eigen"
    "visualization_msgs"
    "geometry_msgs"
    "kirin_msgs")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(kirin_main_executor)
custom_executable(jagariko_marker_publisher)

include_directories(
  include
  gencode
  /usr/include/eigen3
  /usr/local/MATLAB/R2022a/extern/include
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

