##############################################################################
# ROS Environment
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(udp_test_non_qt)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  rospy
  message_runtime
  message_generation
  image_transport
  roslib
)

find_package(Qt5Network REQUIRED)
find_package(Qt5 COMPONENTS Network REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS LIBRARIES udp_test_non_qt
    CATKIN_DEPENDS
    image_transport
    roslib
    roscpp
    std_msgs
    rospy
    DEPENDS
)

set (OpenCV_INClUDE_DIRS /usr/local/include /usr/local/include/opencv2)
set(OpenCV_LIB_DIR /usr/local/lib)
set(Opencv_LIBS opencv_core opencv_highgui opencv_imgcodecs opencv_videoio opencv_imgproc opencv_features2d opencv_flann opencv_calib3d)


##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets Network Multimedia)
find_package(OpenMP REQUIRED)

IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT5_WARP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions( -std=c++11 -fPIC)



##############################################################################
# Sources
##############################################################################

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set( QT_SOURCES
  src/main.cpp
  src/mainwindow.h
  src/mainwindow.cpp
)



##############################################################################
# Binaries
##############################################################################
add_executable(udp_test_non_qt ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(udp_test_non_qt ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Opencv_LIBS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Network

)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
