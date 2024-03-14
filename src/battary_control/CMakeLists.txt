cmake_minimum_required(VERSION 3.8)
project(battary_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
                          
include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/time_count.c)
add_library(src_lib ${SRC_FILE})

add_executable(battary_control_node
          ${PROJECT_SOURCE_DIR}/src/battary_control.cpp)

target_link_libraries(battary_control_node src_lib)        

ament_target_dependencies(
    battary_control_node
    std_msgs
    rclcpp
    sensor_msgs
    geometry_msgs
)

install(
  DIRECTORY launch models worlds 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  battary_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
