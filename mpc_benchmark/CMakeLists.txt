cmake_minimum_required(VERSION 3.20)
project(mpc_benchmark LANGUAGES CXX)

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
find_package(lifecycle_msgs REQUIRED)
find_package(mpc_core REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(tf2_ros REQUIRED)

generate_parameter_library(metrics_recorder_parameters src/metrics_recorder_parameters.yaml)
generate_parameter_library(scenario_runner_parameters src/scenario_runner_parameters.yaml)
generate_parameter_library(ground_truth_relay_parameters src/ground_truth_relay_parameters.yaml)

# Pure C++ metrics (no ROS).
add_library(mpc_benchmark_metrics SHARED src/metrics.cpp)
target_include_directories(mpc_benchmark_metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(mpc_benchmark_metrics PUBLIC mpc_core::mpc_core)

# pluginlib decorator measuring the compute time of any Nav2 controller.
add_library(profiling_controller SHARED src/profiling_controller.cpp)
target_include_directories(profiling_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(profiling_controller PUBLIC
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(nav2_core profiling_controller_plugin.xml)

# Lifecycle components, also available as standalone executables.
add_library(benchmark_components SHARED
  src/metrics_recorder.cpp
  src/scenario_runner.cpp
)
target_link_libraries(benchmark_components PUBLIC
  metrics_recorder_parameters
  scenario_runner_parameters
  mpc_benchmark_metrics
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(benchmark_components
  PLUGIN "mpc_benchmark::MetricsRecorder" EXECUTABLE metrics_recorder)
rclcpp_components_register_node(benchmark_components
  PLUGIN "mpc_benchmark::ScenarioRunner" EXECUTABLE scenario_runner)

# Simulator ground truth (Gazebo only), kept apart so the other components do not need Gazebo.
add_library(ground_truth_relay SHARED src/ground_truth_relay.cpp)
target_include_directories(ground_truth_relay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(ground_truth_relay PUBLIC
  ground_truth_relay_parameters
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
)
rclcpp_components_register_node(ground_truth_relay
  PLUGIN "mpc_benchmark::GroundTruthRelay" EXECUTABLE ground_truth_relay_node)

install(TARGETS
    mpc_benchmark_metrics profiling_controller benchmark_components ground_truth_relay
    metrics_recorder_parameters scenario_runner_parameters ground_truth_relay_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_benchmark.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_metrics test/test_metrics.cpp)
  target_link_libraries(test_metrics mpc_benchmark_metrics)
  ament_add_gtest(test_reference_path test/test_reference_path.cpp)
  target_link_libraries(test_reference_path benchmark_components)
  ament_add_gtest(test_profiling_controller test/test_profiling_controller.cpp TIMEOUT 120)
  target_link_libraries(test_profiling_controller profiling_controller)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  generate_parameter_library geometry_msgs lifecycle_msgs mpc_core nav2_core nav2_msgs
  nav2_ros_common nav_msgs pluginlib rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle
  sensor_msgs std_msgs tf2 tf2_ros gz_msgs_vendor gz-msgs gz_transport_vendor gz-transport
)
ament_package()
