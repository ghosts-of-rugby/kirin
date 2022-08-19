cmake_minimum_required(VERSION 3.5)
project(kirin)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kirin_msgs REQUIRED)

file(GLOB src_files src/*.cpp) 

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} main/${target}.cpp ${src_files})
  ament_target_dependencies(${target} 
    "rclcpp"
    "kirin_msgs")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(manual_controller_node)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../lib/magic_enum/include
)

install(DIRECTORY
    launch
    resources
    urdf
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()

