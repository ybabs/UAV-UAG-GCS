cmake_minimum_required(VERSION 2.8.3)
project(gcs)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
 #add_compile_options(-Wall)
#add_compile_options(-Wpedantic)
#add_compile_options(-Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  message_generation
)

set(Qt5_DIR $ENV{HOME}/Qt/5.14.1/gcc_64/lib/cmake/Qt5)
set(Qt5Location_DIR  "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Location")
set(Qt5Gui_DIR  "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Gui")
set(Qt5Core_DIR  "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Core")
set(Qt5Qml_DIR   "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Qml")    
set(Qt5Positioning_DIR "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Positioning")
set(Qt5OpenGL_DIR  "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5OpenGL")
set(Qt5Sensors_DIR "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Sensors")
set(Qt5Quick_DIR "/home/daniel/Qt/5.14.1/gcc_64/lib/cmake/Qt5Quick")
set(QML_IMPORT_PATH "${QML_IMPORT_PATH};${CATKIN_GLOBAL_LIB_DESTINATION}" )
set(QML_IMPORT_PATH2 "${QML_IMPORT_PATH};${CATKIN_GLOBAL_LIB_DESTINATION}" )

find_package(DJIOSDK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Qml REQUIRED)    
find_package(Qt5Positioning REQUIRED)
find_package(Qt5OpenGL REQUIRED)
#find_package(Qt5Sensors REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Location REQUIRED)

add_message_files(
  FILES
  Waypoint.msg
  Missionparameters.msg
  Action.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dji_qt_gui
  CATKIN_DEPENDS dji_sdk
                 geometry_msgs 
                 nav_msgs
                 roscpp
                 sensor_msgs
                 serial 
                 std_msgs
                 message_runtime
#  DEPENDS system_lib
)

link_directories(/usr/local/lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/..
  include ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)


QT5_WRAP_CPP(QT_MOC_H ${QT_MOC})
QT5_WRAP_UI(QT_FORMS_H ${QT_FORMS})
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)



add_executable(${PROJECT_NAME}_node 
               ${QT_SOURCES} 
               ${QT_RESOURCES_CPP} 
               ${QT_FORMS_H} 
               ${QT_MOC_H})


target_link_libraries(${PROJECT_NAME}_node 
                      ${catkin_LIBRARIES} 
                      ${EIGEN_LIBRARIES}
                      ${DJIOSDK_LIBRARIES}
                      Qt5::Gui
                      Qt5::Qml
                      Qt5::Positioning
                      Qt5::OpenGL
                      Qt5::Quick
                      Qt5::Location 
                      Qt5::Core)

add_dependencies(${PROJECT_NAME}_node 
                 ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
       )
