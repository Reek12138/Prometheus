cmake_minimum_required(VERSION 2.8.3)
project(xsugv_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface xsugv_msgs
  roslaunch roslint roscpp sensor_msgs tf)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS hardware_interface xsugv_msgs roscpp sensor_msgs tf
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_executable(xsugv_node
  src/xsugv_base.cpp
  src/xsugv_hardware.cpp
  src/xsugv_interface.cpp)
target_link_libraries(xsugv_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

roslint_cpp(
  src/xsugv_base.cpp
  src/xsugv_hardware.cpp
  src/xsugv_interface.cpp
  include/xsugv_base/xsugv_hardware.h
  include/xsugv_base/xsugv_interface.h
)

roslaunch_add_file_check(launch)

install(TARGETS xsugv_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
