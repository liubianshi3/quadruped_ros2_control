cmake_minimum_required(VERSION 3.8)
project(dog2_dynamics)

# C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 创建dog2_dynamics库
add_library(${PROJECT_NAME}
  src/dog2_model.cpp
)

# pinocchio >= 4.0 requires the imported target: it carries both the normal
# include dir and the deprecated compat include dir (pinocchio/deprecated).
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# 创建测试可执行文件
add_executable(test_dog2_model
  src/test_dog2_model.cpp
)

target_link_libraries(test_dog2_model
  ${PROJECT_NAME}
  pinocchio::pinocchio
)

ament_target_dependencies(test_dog2_model
  ament_index_cpp
  rclcpp
)

# 安装
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS test_dog2_model
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(pinocchio Eigen3 rclcpp ament_index_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
