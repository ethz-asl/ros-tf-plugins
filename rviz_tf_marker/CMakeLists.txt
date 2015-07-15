cmake_minimum_required(VERSION 2.8.3)
project(rviz_tf_marker)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    tf2
    tf2_msgs
    tf2_ros
    tf2_geometry_msgs
    roscpp
    rviz
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rviz_tf_marker_SRCS
  src/tf_marker.cpp
  src/tf_marker_arrow.cpp
  src/tf_marker_control.cpp
  src/tf_marker_crosshair.cpp
  src/tf_marker_description.cpp
  src/tf_marker_disc.cpp
  src/tf_marker_display.cpp
)

set(rviz_tf_marker_HDRS
  include/rviz_tf_marker/tf_marker.h
  include/rviz_tf_marker/tf_marker_arrow.h
  include/rviz_tf_marker/tf_marker_control.h
  include/rviz_tf_marker/tf_marker_crosshair.h
  include/rviz_tf_marker/tf_marker_description.h
  include/rviz_tf_marker/tf_marker_disc.h
  include/rviz_tf_marker/tf_marker_display.h
)

set(rviz_tf_marker_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES
    rviz_tf_marker
  CATKIN_DEPENDS 
    geometry_msgs
    tf2
    tf2_msgs
    tf2_ros
    roscpp
    rviz
)

qt4_wrap_cpp(rviz_tf_marker_MOCS ${rviz_tf_marker_HDRS})

include_directories(
  ${rviz_tf_marker_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_library(rviz_tf_marker
  ${rviz_tf_marker_SRCS} 
  ${rviz_tf_marker_MOCS}
)

target_link_libraries(rviz_tf_marker
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
)

find_package(class_loader)
class_loader_hide_library_symbols(rviz_tf_marker)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rviz_tf_marker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rviz_tf_marker
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rviz_tf_marker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
