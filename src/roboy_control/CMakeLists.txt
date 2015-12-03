## FIND CATKIN/ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## FIND QT
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

add_subdirectory(src)

message(Include Dirs:\n${ROBOY_INCLUDE_DIRS})
message(Headers Found:\n${ROBOY_HEADERS})
message(Sources Found:\n${ROBOY_SOURCES})
message(UI-Files Found:\n${ROBOY_UI_FILES})

## Dummy target to add non-source files to Qt-Project Viewer
file(GLOB ETC ${CMAKE_CURRENT_SOURCE_DIR}/etc/*)
file(GLOB DB  ${CMAKE_CURRENT_SOURCE_DIR}/database/*)
add_custom_target(dummy_target SOURCES ${ROBOY_HEADERS} ${ETC} ${DB})

## Generate Qt-Files (Mocs for Headers & UI-Fieles)
qt4_wrap_cpp(MOC_HDRS ${ROBOY_HEADERS})
qt4_wrap_ui(MOC_UI ${ROBOY_UI_FILES})

## Build Target
include_directories(
        ${ROBOY_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${QT_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(roboy_control_node ${ROBOY_SOURCES} ${MOC_HDRS} ${MOC_UI})

target_link_libraries(roboy_control_node ${catkin_LIBRARIES} ${QT_LIBRARIES})

