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
find_package(rosidl_default_runtime REQUIRED)

# Declare service files
set(srv_files
  "srv/ChangeString.srv"
)

# Generate the ROS interface packages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

# Declare a C++ executable
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(change_string_client src/change_string_client.cpp)

ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(change_string_client rclcpp std_msgs)

# Get the typesupport target for the rosidl interfaces
rosidl_get_typesupport_target(cpp_ts_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the executables against the typesupport target
target_link_libraries(talker "${cpp_ts_target}")
target_link_libraries(listener "${cpp_ts_target}")
target_link_libraries(change_string_client "${cpp_ts_target}")

# Install the executables
install(TARGETS
  talker
  listener
  change_string_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
