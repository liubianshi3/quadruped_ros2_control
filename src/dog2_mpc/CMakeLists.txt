cmake_minimum_required(VERSION 3.8)
project(dog2_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dog2_dynamics REQUIRED)
find_package(dog2_interfaces REQUIRED)
# pinocchio >= 4.0: headers must come from the imported target, which also
# carries the include/pinocchio/deprecated compat path used by dog2_dynamics.
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find OSQP
find_package(PkgConfig REQUIRED)
pkg_check_modules(OSQP REQUIRED osqp)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OSQP_INCLUDE_DIRS}
)

# Extended SRBD Model library (包含滑动副)
add_library(extended_srbd_model
  src/extended_srbd_model.cpp
)
target_include_directories(extended_srbd_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(extended_srbd_model
  srbd_model
)
ament_target_dependencies(extended_srbd_model
  Eigen3
)

# SRBD Model library
add_library(srbd_model
  src/srbd_model.cpp
)
target_include_directories(srbd_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(srbd_model
  Eigen3
)

# Sliding Constraints library
add_library(sliding_constraints
  src/sliding_constraints.cpp
)
target_include_directories(sliding_constraints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sliding_constraints
  Eigen3
)

# Boundary Constraints library
add_library(boundary_constraints
  src/boundary_constraints.cpp
)
target_include_directories(boundary_constraints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(boundary_constraints
  Eigen3
)

# OSQP Interface library
add_library(osqp_interface SHARED
  src/osqp_interface.cpp
)
target_include_directories(osqp_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OSQP_INCLUDE_DIRS}
)
target_link_libraries(osqp_interface
  ${OSQP_LIBRARIES}
)
ament_target_dependencies(osqp_interface
  Eigen3
)

# Flat-ground receding-horizon body controller and constrained contact-force
# allocator. Kept separate from the legacy crossing MPC.
add_library(flat_locomotion_mpc
  src/flat_locomotion_mpc.cpp
)
target_include_directories(flat_locomotion_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OSQP_INCLUDE_DIRS}
)
target_link_libraries(flat_locomotion_mpc
  osqp_interface
  ${OSQP_LIBRARIES}
)
ament_target_dependencies(flat_locomotion_mpc
  Eigen3
)

# Crossing State Machine library
add_library(crossing_state_machine
  src/crossing_state_machine.cpp
)
target_include_directories(crossing_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(crossing_state_machine
  Eigen3
)

# Hybrid Gait Generator library
add_library(hybrid_gait_generator
  src/hybrid_gait_generator.cpp
)
target_include_directories(hybrid_gait_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_gait_generator
  crossing_state_machine
)
ament_target_dependencies(hybrid_gait_generator
  Eigen3
)

# Trajectory Generator library
add_library(trajectory_generator
  src/trajectory_generator.cpp
)
target_include_directories(trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_generator
  crossing_state_machine
  hybrid_gait_generator
)
ament_target_dependencies(trajectory_generator
  Eigen3
)

# Contact Detector library
add_library(contact_detector
  src/contact_detector.cpp
)
target_include_directories(contact_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(contact_detector
  Eigen3
)

# MPC Controller library
add_library(mpc_controller
  src/mpc_controller.cpp
)
target_include_directories(mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_controller
  extended_srbd_model
  srbd_model
  sliding_constraints
  boundary_constraints
  osqp_interface
  crossing_state_machine
  hybrid_gait_generator
)
ament_target_dependencies(mpc_controller
  Eigen3
)

# Install libraries
install(TARGETS
  extended_srbd_model
  srbd_model
  sliding_constraints
  boundary_constraints
  osqp_interface
  flat_locomotion_mpc
  crossing_state_machine
  hybrid_gait_generator
  trajectory_generator
  contact_detector
  mpc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Complete MPC Node executable (with trajectory generation and contact detection)
add_executable(mpc_node_complete
  src/mpc_node_complete.cpp
)
target_include_directories(mpc_node_complete PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mpc_node_complete
  extended_srbd_model
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
  crossing_state_machine
  hybrid_gait_generator
  trajectory_generator
  contact_detector
  pinocchio::pinocchio
)
ament_target_dependencies(mpc_node_complete
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  Eigen3
  dog2_dynamics
  dog2_interfaces
)

# Flat-ground controller entrypoint. It deliberately does not link the
# crossing state machine or the legacy post-QP force shapers.
add_executable(flat_mpc_node
  src/flat_mpc_node.cpp
)
target_include_directories(flat_mpc_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(flat_mpc_node
  flat_locomotion_mpc
  pinocchio::pinocchio
)
ament_target_dependencies(flat_mpc_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  Eigen3
  dog2_dynamics
  dog2_interfaces
)

# Install executables
install(TARGETS mpc_node_complete flat_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Test executable for crossing system
add_executable(test_crossing_system
  test/test_crossing_system.cpp
)
target_include_directories(test_crossing_system PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_crossing_system
  crossing_state_machine
  hybrid_gait_generator
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
)
ament_target_dependencies(test_crossing_system
  Eigen3
)

# Test executable for MPC control loop
add_executable(test_mpc_crossing_loop
  test/test_mpc_crossing_loop.cpp
)
target_include_directories(test_mpc_crossing_loop PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_mpc_crossing_loop
  crossing_state_machine
  hybrid_gait_generator
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
)
ament_target_dependencies(test_mpc_crossing_loop
  Eigen3
)

# Test executable for basic MPC
add_executable(test_basic_mpc
  test/test_basic_mpc.cpp
)
target_include_directories(test_basic_mpc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_basic_mpc
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
)
ament_target_dependencies(test_basic_mpc
  Eigen3
)

# Test executable for simple tracking
add_executable(test_simple_tracking
  test/test_simple_tracking.cpp
)
target_include_directories(test_simple_tracking PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_simple_tracking
  osqp_interface
)
ament_target_dependencies(test_simple_tracking
  Eigen3
)

# Test executable for MPC constraints
add_executable(test_mpc_constraints
  test/test_mpc_constraints.cpp
)
target_include_directories(test_mpc_constraints PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_mpc_constraints
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
  crossing_state_machine
  hybrid_gait_generator
)
ament_target_dependencies(test_mpc_constraints
  Eigen3
)

# Test executable for sliding constraints
add_executable(test_sliding_constraints
  test/test_sliding_constraints.cpp
)
target_include_directories(test_sliding_constraints PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_sliding_constraints
  extended_srbd_model
  srbd_model
  sliding_constraints
)
ament_target_dependencies(test_sliding_constraints
  Eigen3
)

# Test executable for 16D MPC
add_executable(test_16d_mpc
  test/test_16d_mpc.cpp
)
target_include_directories(test_16d_mpc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_16d_mpc
  extended_srbd_model
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
  crossing_state_machine
  hybrid_gait_generator
)
ament_target_dependencies(test_16d_mpc
  Eigen3
)

# Test executable for 16D MPC with sliding constraints
add_executable(test_16d_mpc_with_sliding
  test/test_16d_mpc_with_sliding.cpp
)
target_include_directories(test_16d_mpc_with_sliding PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_16d_mpc_with_sliding
  extended_srbd_model
  mpc_controller
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
  crossing_state_machine
  hybrid_gait_generator
)
ament_target_dependencies(test_16d_mpc_with_sliding
  Eigen3
)

# Test executable for crossing basic functionality
add_executable(test_crossing_basic
  test/test_crossing_basic.cpp
)
target_include_directories(test_crossing_basic PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_link_libraries(test_crossing_basic
  crossing_state_machine
  hybrid_gait_generator
)
ament_target_dependencies(test_crossing_basic
  Eigen3
)

# Test executable for crossing MPC integration
add_executable(test_crossing_mpc
  test/test_crossing_mpc.cpp
)
target_include_directories(test_crossing_mpc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_link_libraries(test_crossing_mpc
  mpc_controller
  crossing_state_machine
  hybrid_gait_generator
  extended_srbd_model
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
)
ament_target_dependencies(test_crossing_mpc
  Eigen3
)

# Test executable for simplified crossing MPC (debugging)
add_executable(test_crossing_mpc_simple
  test/test_crossing_mpc_simple.cpp
)
target_include_directories(test_crossing_mpc_simple PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_link_libraries(test_crossing_mpc_simple
  mpc_controller
  crossing_state_machine
  hybrid_gait_generator
  extended_srbd_model
  osqp_interface
  srbd_model
  sliding_constraints
  boundary_constraints
)
ament_target_dependencies(test_crossing_mpc_simple
  Eigen3
)

# Install test executables
install(TARGETS
  test_crossing_system
  test_mpc_crossing_loop
  test_basic_mpc
  test_simple_tracking
  test_mpc_constraints
  test_sliding_constraints
  test_16d_mpc
  test_16d_mpc_with_sliding
  test_crossing_basic
  test_crossing_mpc
  test_crossing_mpc_simple
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  add_executable(test_mpc_urdf_model_data
    test/test_mpc_urdf_model_data.cpp
  )
  target_link_libraries(test_mpc_urdf_model_data
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_mpc_urdf_model_data
    Eigen3
    dog2_dynamics
    ament_index_cpp
  )
  add_test(NAME test_mpc_urdf_model_data COMMAND test_mpc_urdf_model_data)
  add_executable(test_flat_locomotion_mpc
    test/test_flat_locomotion_mpc.cpp
  )
  target_link_libraries(test_flat_locomotion_mpc
    flat_locomotion_mpc
  )
  ament_target_dependencies(test_flat_locomotion_mpc
    Eigen3
  )
  add_test(NAME test_flat_locomotion_mpc COMMAND test_flat_locomotion_mpc)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
