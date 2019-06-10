cmake_minimum_required(VERSION 2.8.3)
project(cv_test)

add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  #catkin_make
  roscpp
  rospy
    message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
#    riki_msgs
)
find_package(Qt5  REQUIRED COMPONENTS  Widgets  )

find_package(VISP REQUIRED)

find_package(OpenCV REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/mainwindow.cpp
  src/qnode.cpp
  src/display_node.cpp
  src/visionalgorithm.cpp
)

set(FORMS
  src/mainwindow.ui
)



catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS}
)

 add_executable(cv_disp ${SOURCES} ${FORMS} )
 target_link_libraries(cv_disp  Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})

















