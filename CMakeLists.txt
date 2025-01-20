cmake_minimum_required(VERSION 3.5)
project(ros2_controller)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Personal packages
find_package(ros2_input_msgs REQUIRED)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp ros2_input_msgs)
target_include_directories(talker PUBLIC ${CON_UTILS_DIR}/include)
target_link_directories(talker PUBLIC ${CON_UTILS_DIR}/build/lib)
target_link_libraries(talker controller_utilities)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp ros2_input_msgs)
target_include_directories(listener PUBLIC ${CON_UTILS_DIR}/include)
target_link_directories(listener PUBLIC ${CON_UTILS_DIR}/build/lib)
target_link_libraries(listener controller_utilities)

install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})

ament_package()
