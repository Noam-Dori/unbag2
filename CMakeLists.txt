cmake_minimum_required(VERSION 3.5)
project(unbag2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

add_library(unbag_lib src/unbag_node.cpp src/pipe.cpp src/generic_subscription.cpp src/wild_msg.cpp)
target_include_directories(unbag_lib PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
ament_target_dependencies(unbag_lib rclcpp pluginlib rosbag2_cpp)
target_link_libraries(unbag_lib ${Boost_LIBRARIES})

add_executable(unbag2 src/unbag_main.cpp)
target_include_directories(unbag2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(unbag2 rclcpp pluginlib)
target_link_libraries(unbag2 unbag_lib)

install(TARGETS unbag2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
