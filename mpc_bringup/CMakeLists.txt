cmake_minimum_required(VERSION 3.20)
project(mpc_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Full Nav2 stack on the loopback simulator, once per MPC variant.
  foreach(_controller mpc_rti mpc_ltv)
    add_launch_test(test/test_navigation.launch.py
      TARGET test_navigation_${_controller}
      ARGS "controller:=${_controller}"
      TIMEOUT 120
    )
  endforeach()
endif()

ament_package()
