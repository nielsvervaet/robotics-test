cmake_minimum_required(VERSION 3.8)
project(listener_jointstates_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

function(add_demo_dependencies library)
  target_compile_definitions(${library}
    PRIVATE "LISTENER_JOINTSTATES_CPP_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rclcpp_components"
    "rcutils"
    "std_msgs"
	  "sensor_msgs")
endfunction()

# Add library
add_library(listeners_library SHARED
  src/listener_jointstates.cpp)

add_demo_dependencies(listeners_library)

rclcpp_components_register_node(listeners_library
  PLUGIN "demo_nodes_cpp::Listener_JointStates"
  EXECUTABLE listener_jointstates_cpp)

install(TARGETS
  listeners_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS listeners_library
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()