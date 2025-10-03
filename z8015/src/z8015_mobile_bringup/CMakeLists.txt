cmake_minimum_required(VERSION 3.8)
project(z8015_mobile_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(CppLinuxSerial REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(
        include
        /root/ws/install/serial/include  # 수동으로 serial 패키지의 헤더 디렉토리 추가
)
link_directories(/root/ws/install/serial/lib)


add_executable(z8015_driver src/z8015_driver.cpp)
target_link_libraries(z8015_driver CppLinuxSerial::CppLinuxSerial)
target_link_libraries(z8015_driver serial)

# Add dependencies
ament_target_dependencies(z8015_driver
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        tf2_ros
        tf2_geometry_msgs
        serial
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
        z8015_driver
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

# Install config files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
