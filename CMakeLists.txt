cmake_minimum_required(VERSION 3.1)

project(
  RosImuSensor
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# To use ros
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs)
message("=========== Catkin include directories : ${catkin_INCLUDE_DIRS}")
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

# Configure and install the plugin configuration
configure_file(etc/RosImuSensor.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/RosImuSensor.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RosImuSensor.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC 
    src/RosImuSensor.cpp
    )

set(plugin_HDR 
    src/RosImuSensor.h
    include/utils/ROSSubscriber.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(RosImuSensor "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(RosImuSensor PUBLIC mc_rtc::mc_rtc_ros)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
