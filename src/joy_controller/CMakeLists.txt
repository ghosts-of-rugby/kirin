cmake_minimum_required(VERSION 3.5)
project(joy_controller)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

file(GLOB src_files src/*.cpp) 

add_library(PACKAGE_LIBRARIES ${src_files})
target_include_directories(PACKAGE_LIBRARIES PUBLIC
  include
  ${CMAKE_SOURCE_DIR}/../../lib/magic_enum/include
)


# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# talkerノードのビルド設定
custom_executable(talker)


install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
