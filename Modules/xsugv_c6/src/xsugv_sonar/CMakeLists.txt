cmake_minimum_required(VERSION 2.8.3)
project(xsugv_sonar)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_executable(xsugv_sonar_node src/xsugv_sonar.cpp)

target_link_libraries(xsugv_sonar_node ${catkin_LIBRARIES})

install(TARGETS xsugv_sonar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

