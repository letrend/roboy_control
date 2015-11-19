cmake_minimum_required(VERSION 2.8.3)
project(roboy_control)


#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

## SETUP VARIABLES
set(CMAKE_CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)



## FIND CATKIN/ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## FIND QT
find_package(Qt4 COMPONENTS QtCore REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

## FIND HEADERS AND SOURCES
file(GLOB_RECURSE SRCS
            ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/model/*.cpp
)
file(GLOB_RECURSE HDRS
            ${CMAKE_CURRENT_SOURCE_DIR}/*.h
            ${CMAKE_CURRENT_SOURCE_DIR}/model/*.h
)

message(HEADERS FOUND: ${HDRS})
message(SOURCES FOUND: ${SRCS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${QT_INCLUDE_DIR}
)

qt4_wrap_cpp(moc ${HDRS})

add_executable(roboy_control_node ${SRCS} ${HDRS})

set_property(TARGET roboy_control_node PROPERTY CXX_STANDARD 11)
set_property(TARGET roboy_control_node APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )

target_link_libraries(roboy_control_node ${catkin_LIBRARIES} ${QT_LIBRARIES} ${QT_QTNETWORK_LIBRARY})
