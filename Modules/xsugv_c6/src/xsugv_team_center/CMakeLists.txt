cmake_minimum_required(VERSION 3.0.2)
project(xsugv_team_center)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS message_generation rviz roscpp nav_msgs std_msgs tf2 map_server)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

add_service_files(
  FILES RosMasterRelayComm.srv
)

generate_messages(  
  DEPENDENCIES nav_msgs std_msgs roscpp
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs nav_msgs map_server message_runtime
)

###########
## Build ##
###########
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui QtNetwork QtConcurrent)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Network)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets Qt5::Network)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  include
  ${YAMLCPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
)

file(GLOB_RECURSE XSUGV_RVIZ_PLUGINS_SRC
 "${CMAKE_CURRENT_SOURCE_DIR}/src/rviz_plugins/*.cpp"
)
file(GLOB_RECURSE XSUGV_RVIZ_PLUGINS_INC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/rviz_plugins/*.h"
)

add_library(xsugv_rviz_plugins
  ${XSUGV_RVIZ_PLUGINS_INC}
  ${XSUGV_RVIZ_PLUGINS_SRC}
)

## Specify libraries to link a library or executable target against
target_link_libraries(xsugv_rviz_plugins
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

add_executable(xsugv_ros_master_relay_node
  src/ros_master_relay/ros_master_relay.cpp
  src/ros_master_relay/ros_master_relay_node.cpp
)

target_link_libraries(xsugv_ros_master_relay_node 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  map_server_image_loader
)


#############
## Install ##
#############
install(TARGETS xsugv_ros_master_relay_node xsugv_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  xsugv_rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

