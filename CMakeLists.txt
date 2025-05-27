cmake_minimum_required(VERSION 3.8)
project(rs2)

# Enable compile commands export for VSCode IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find common dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Find moveIt dependencies
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

# Enable automoc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#common include directories
set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------
# (GUI)
# ------------------------------------------------------

add_executable(gui
  src/main.cpp
  src/GUI/GUI.cpp
  src/GUI/RobotNode.cpp
)

target_include_directories(gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

target_include_directories(gui PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Link libraries
ament_target_dependencies(gui
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)

target_link_libraries(gui
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

# ------------------------------------------------------
# ROBOT_MANIPULATION
# ------------------------------------------------------

# ------------------------------------------------------
# (boardPos)
# ------------------------------------------------------

add_library(boardpos_lib
  src/robot_manipulation/boardPos.cpp
)

target_include_directories(boardpos_lib PUBLIC
  ${COMMON_INCLUDE_DIRS}
)

ament_target_dependencies(boardpos_lib 
  rclcpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
)

add_executable(boardPos
  src/robot_manipulation/boardTester.cpp
)

target_include_directories(boardPos PUBLIC
  ${COMMON_INCLUDE_DIRS}
)

ament_target_dependencies(boardPos
  rclcpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
)

target_link_libraries(boardPos
  boardpos_lib
  robotControl_lib
)

# ------------------------------------------------------
# (robotControl)
# ------------------------------------------------------

add_library(robotControl_lib
  src/robot_manipulation/robotControl.cpp
)

target_include_directories(robotControl_lib PUBLIC
  ${COMMON_INCLUDE_DIRS}
)

ament_target_dependencies(robotControl_lib
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
)

add_executable(robotControl
  src/robot_manipulation/main.cpp
)

ament_target_dependencies(robotControl
  rclcpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
)

target_include_directories(robotControl PUBLIC 
  ${COMMON_INCLUDE_DIRS}
)

target_link_libraries(robotControl
  robotControl_lib
  boardpos_lib
)

# ------------------------------------------------------
# (Python)
# ------------------------------------------------------

ament_python_install_package(computer_vision)
install(PROGRAMS
  # This points to the Python entry point
  computer_vision/chess_core.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------
# (Installation)
# ------------------------------------------------------

install(
  TARGETS 
  gui
  boardPos
  robotControl
  boardpos_lib
  #LIBRARY LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  OPTIONAL
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

ament_package()

