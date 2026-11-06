cmake_minimum_required(VERSION 3.20)
project(mpc_core LANGUAGES CXX)

# C++20 everywhere in the repository (required by the Nav2 headers in Lyrical).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(MPC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
  if(MPC_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)

# Pure C++ library: no ROS dependency, only Eigen (public) and OSQP (private, behind a pimpl).
add_library(${PROJECT_NAME} SHARED
  src/baseline_solver.cpp
  src/closed_loop.cpp
  src/linearized_mpc_solver.cpp
  src/obstacle_constraints.cpp
  src/osqp_solver.cpp
  src/path_generators.cpp
  src/reference_builder.cpp
  src/rotate_to_heading.cpp
  src/solver_factory.cpp
  src/tracking_qp.cpp
  src/unicycle_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PRIVATE osqp::osqp
)

# Offline timing study of the solvers (closed loop on a figure eight), used for the README.
add_executable(mpc_core_timing tools/mpc_core_timing.cpp)
target_link_libraries(mpc_core_timing ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mpc_core_timing RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(_tests
    test_unicycle_model
    test_reference_builder
    test_obstacle_constraints
    test_tracking_qp
    test_osqp_solver
    test_rotate_to_heading
    test_mpc_solvers
  )
  foreach(_test ${_tests})
    ament_add_gtest(${_test} test/${_test}.cpp TIMEOUT 120)
    target_link_libraries(${_test} ${PROJECT_NAME})
    target_compile_definitions(${_test} PRIVATE
      MPC_CORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endforeach()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# osqp is a private dependency, but consumers still need it to link the shared library.
ament_export_dependencies(eigen3_cmake_module Eigen3 osqp_vendor osqp)
ament_package()
