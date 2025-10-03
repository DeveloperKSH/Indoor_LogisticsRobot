cmake_minimum_required(VERSION 3.5)
project(uron)
# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(EIGEN3_INCLUDE_DIRS /usr/include/eigen3)


# Add executable
add_executable(uron_node src/main.cpp)
target_include_directories(uron_node
        PRIVATE
        include
        ${EIGEN3_INCLUDE_DIRS}
        )
# Add dependencies
ament_target_dependencies(uron_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        )
# Install executable
install(TARGETS uron_node
        DESTINATION lib/${PROJECT_NAME}
        )
# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )
set(CATKIN_INSTALL_INTO_PREFIX_ROOT OFF)

# Register package
ament_package()