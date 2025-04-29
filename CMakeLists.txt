cmake_minimum_required(VERSION 3.8)
project(rs2)

# Enable compile commands export for VSCode IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Find executors
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automoc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Add sources to the executable
add_executable(rs2
  src/main.cpp
  src/GUI/GUI.cpp
)

add_executable(boardPos
  src/robot_manipulation/boardPosTester.cpp
  src/robot_manipulation/boardPos.cpp
)

add_executable(ur3e_mtc_demo
  src/ur3e_mtc_demo.cpp
)

# Include directories
target_include_directories(rs2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

target_include_directories(boardPos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(rs2 PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Link libraries
ament_target_dependencies(rs2
  rclcpp
  rclcpp_components
  std_msgs
)

ament_target_dependencies(boardPos
  rclcpp
  std_msgs
)

ament_target_dependencies(ur3e_mtc_demo
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  tf2_geometry_msgs
  tf2_eigen
)

target_link_libraries(rs2
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  rs2
  boardPos
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY 
  include/
  DIRECTORY launch/
  DESTINATION include/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

ament_package()