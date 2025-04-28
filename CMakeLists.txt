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

target_link_libraries(rs2
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

# Install targets
install(TARGETS
  rs2
  boardPos
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
  OPTIONAL
)

ament_package()