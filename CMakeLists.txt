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
find_package(tf2_ros REQUIRED)  # Added for TF2 support
find_package(tf2_geometry_msgs REQUIRED)  # Added for TF2 geometry messages

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

ament_target_dependencies(talker rclcpp std_msgs tf2_ros tf2_geometry_msgs)
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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # un-comment the following lines skip linters (TRUE = skip that test)
  # set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  # set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # set(ament_cmake_lint_cmake_FOUND TRUE)

  add_custom_target(
    cpplint_custom
    COMMAND cpplint --filter=-build/include_order --recursive ${CMAKE_SOURCE_DIR}/src
    COMMENT "Running cpplint with custom filters"
  )

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/test_talker.cpp)
  ament_add_gtest(test_talker ${TESTFILES})
  ament_target_dependencies(test_talker rclcpp std_msgs tf2_ros tf2_geometry_msgs rosidl_default_runtime)
endif()

ament_package()
