cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare service files
set(srv_files
  "srv/ChangeString.srv"
)

# Declare a C++ executable
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

# Generate and link against the ROS interface packages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_ts_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker ${cpp_ts_lib})
target_link_libraries(listener ${cpp_ts_lib})

# Link against the ROS interface packages
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)

# Install targets
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
