cmake_minimum_required(VERSION 3.8)
project(dog2_wbc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dog2_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dog2_dynamics REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# WBC Controller library
add_library(wbc_controller
  src/wbc_controller.cpp
)
target_include_directories(wbc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wbc_controller
  pinocchio::pinocchio
)
ament_target_dependencies(wbc_controller
  Eigen3
  dog2_dynamics
)

# Complete WBC Node executable
add_executable(wbc_node_complete
  src/wbc_node_complete.cpp
)
target_link_libraries(wbc_node_complete
  wbc_controller
  pinocchio::pinocchio
)
ament_target_dependencies(wbc_node_complete
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  dog2_interfaces
  Eigen3
  dog2_dynamics
)

# Install libraries
install(TARGETS wbc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS wbc_node_complete
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  add_executable(test_wbc_urdf_jacobian
    test/test_wbc_urdf_jacobian.cpp
  )
  target_link_libraries(test_wbc_urdf_jacobian
    wbc_controller
  )
  ament_target_dependencies(test_wbc_urdf_jacobian
    Eigen3
    dog2_dynamics
    ament_index_cpp
  )
  add_test(NAME test_wbc_urdf_jacobian COMMAND test_wbc_urdf_jacobian)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
