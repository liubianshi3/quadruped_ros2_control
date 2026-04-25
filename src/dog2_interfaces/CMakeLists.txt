cmake_minimum_required(VERSION 3.8)
project(dog2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/BodyCommand.msg"
  "msg/ContactPhase.msg"
  "msg/ContactState.msg"
  "msg/ControllerMode.msg"
  "msg/FootState.msg"
  "msg/FootTrajectory.msg"
  "msg/GRFReference.msg"
  "msg/GaitCommand.msg"
  "msg/MPCDebug.msg"
  "msg/MPCHorizon.msg"
  "msg/RobotState.msg"
  "msg/SystemMode.msg"
  "msg/TaskStatus.msg"
  "msg/WBCDebug.msg"
)

set(srv_files
  "srv/SetControllerMode.srv"
  "srv/SetGait.srv"
  "srv/SetRailMode.srv"
  "srv/TriggerAction.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
