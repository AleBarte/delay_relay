cmake_minimum_required(VERSION 3.8)
project(delay_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(sensor_msgs REQUIRED)

# ============================================================================
# 1. delay_relay_node
# ============================================================================
add_library(delay_relay_lib SHARED
  src/delay_relay_node.cpp
)
target_include_directories(delay_relay_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(delay_relay_lib rclcpp std_msgs sensor_msgs)

add_executable(delay_relay_node src/main.cpp)
target_link_libraries(delay_relay_node delay_relay_lib)
ament_target_dependencies(delay_relay_node rclcpp)

# ============================================================================
# 2. init_node
# ============================================================================
add_library(init_lib SHARED
  src/init_node.cpp
)
target_include_directories(init_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(init_lib rclcpp std_msgs sensor_msgs)

add_executable(init_node src/main_init.cpp)
target_link_libraries(init_node init_lib)
ament_target_dependencies(init_node rclcpp)

# ============================================================================
# Install
# ============================================================================
install(TARGETS
  delay_relay_lib delay_relay_node
  init_lib        init_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch    DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
