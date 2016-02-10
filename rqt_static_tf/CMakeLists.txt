cmake_minimum_required(VERSION 2.8.3)
project(rqt_static_tf)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    tf2
    tf2_msgs
    tf2_ros
    roscpp
    rqt_gui
    rqt_gui_cpp
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

find_package(Eigen3 REQUIRED)

include(${QT_USE_FILE})

catkin_python_setup()

set(rqt_static_tf_SRCS
  src/static_tf_plugin.cpp
)

set(rqt_static_tf_HDRS
  include/rqt_static_tf/static_tf_plugin.h
)

set(rqt_static_tf_UIS
  src/static_tf_plugin.ui
)

set(rqt_static_tf_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES
    rqt_static_tf
  CATKIN_DEPENDS
    tf2
    tf2_msgs
    tf2_ros
    roscpp
    rqt_gui
    rqt_gui_cpp
)

qt4_wrap_cpp(rqt_static_tf_MOCS ${rqt_static_tf_HDRS})
qt4_wrap_ui(rqt_static_tf_UIS_H ${rqt_static_tf_UIS})

include_directories(
  ${rqt_static_tf_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(rqt_static_tf
  ${rqt_static_tf_SRCS}
  ${rqt_static_tf_MOCS}
  ${rqt_static_tf_UIS_H}
)

target_link_libraries(rqt_static_tf
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(rqt_static_tf)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_static_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_static_tf
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_static_tf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
