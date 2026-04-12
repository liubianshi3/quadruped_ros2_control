cmake_minimum_required(VERSION 3.8)
project(dog2_description)

find_package(ament_cmake REQUIRED)

option(DOG2_ENABLE_URDF_SHIFT_BOUNDARY_CHECK "Enable URDF shift boundary validation test" ON)

# Find xacro for URDF generation
find_program(XACRO xacro)
if(NOT XACRO)
  message(WARNING "xacro not found. Runtime robot_description expansion and URDF boundary checks may be unavailable.")
else()
  # Canonical robot description lives in urdf/dog2.urdf.xacro.
  # Committed static URDF snapshots are intentionally not generated at build
  # time. Export a snapshot explicitly when you need one for inspection, e.g.:
  #   xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/dog2.urdf.xacro \
  #     controllers_yaml:=${CMAKE_CURRENT_SOURCE_DIR}/config/ros2_controllers.yaml \
  #     -o /tmp/dog2.urdf
endif()

install(
  DIRECTORY urdf meshes launch rviz config scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  FILES config/migration_stage2_stand.json
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts as executables
install(
  PROGRAMS
    scripts/auto_stand_node.py
    scripts/spider_joint_smoke_test.py
    scripts/spider_keyboard_control.py
    scripts/spider_ik_stand.py
    scripts/spider_basic_motion_controller.py
    scripts/check_urdf_shift_boundary.py
    scripts/export_dog2_urdf.py
    scripts/verify_stage2_rail_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING AND DOG2_ENABLE_URDF_SHIFT_BOUNDARY_CHECK)
  find_program(PYTHON3_EXECUTABLE python3)
  find_program(XACRO_EXECUTABLE xacro)

  if(PYTHON3_EXECUTABLE AND XACRO_EXECUTABLE)
    add_test(
      NAME dog2_urdf_shift_boundary_check
      COMMAND
        ${PYTHON3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_urdf_shift_boundary.py
        ${CMAKE_CURRENT_SOURCE_DIR}/urdf/dog2.urdf.xacro
        --strict
    )
    add_test(
      NAME dog2_stage2_rail_geometry_check
      COMMAND
        ${PYTHON3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/verify_stage2_rail_geometry.py
        --xacro
        ${CMAKE_CURRENT_SOURCE_DIR}/urdf/dog2.urdf.xacro
        --golden
        ${CMAKE_CURRENT_SOURCE_DIR}/config/migration_stage2_stand.json
    )
  else()
    message(WARNING "Skip dog2_urdf_shift_boundary_check: python3 or xacro not found")
  endif()
endif()

ament_package()
