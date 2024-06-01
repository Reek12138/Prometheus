cmake_minimum_required(VERSION 2.8.3)
project(xsugv_team_interface)

find_package(catkin REQUIRED COMPONENTS roslaunch roslint roscpp sensor_msgs xsugv_msgs tf pcl_ros tf2 tf2_ros)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs xsugv_msgs tf pcl_ros tf2 tf2_ros
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;")

add_executable(xsugv_team_node src/xsugv_team_node.cpp src/xsugv_team_interface.cpp)

target_link_libraries(xsugv_team_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


roslaunch_add_file_check(launch)

install(TARGETS xsugv_team_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
