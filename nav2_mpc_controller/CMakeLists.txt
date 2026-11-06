cmake_minimum_required(VERSION 3.20)
project(nav2_mpc_controller LANGUAGES CXX)

# C++20 is required by the Nav2 headers in Lyrical (std::atomic<std::shared_ptr>).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(MPC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor)
  if(MPC_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_core REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Parameter declaration, validation and runtime updates generated from YAML.
generate_parameter_library(mpc_controller_parameters src/mpc_controller_parameters.yaml)

add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/costmap_tools.cpp
  src/mpc_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mpc_controller_parameters
  mpc_core::mpc_core
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(nav2_core mpc_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME} mpc_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})
  ament_add_gtest(test_mpc_controller test/test_mpc_controller.cpp TIMEOUT 120)
  target_link_libraries(test_mpc_controller ${PROJECT_NAME})
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  generate_parameter_library geometry_msgs mpc_core nav2_core nav2_costmap_2d nav2_ros_common
  nav_msgs pluginlib rclcpp rclcpp_lifecycle std_msgs visualization_msgs
)
ament_package()
