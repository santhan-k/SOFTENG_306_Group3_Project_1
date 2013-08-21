cmake_minimum_required(VERSION 2.8.3)
project(this_laser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Build the laser_scan_publisher executable
add_executable(laser_scan_listener src/laser_scan_listener.cpp)

# Add a build order dependency on sensor_msgs
# This ensures that sensor_msgs' msg headers are built before your executable
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(laser_scan_listener ${sensor_msgs_EXPORTED_TARGETS})
endif()
# Link against the catkin_LIBRARIES
target_link_libraries(laser_scan_listener ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS laser_scan_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
