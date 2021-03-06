## Find CATKIN/ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Find QT
## Minimum version requirements
set(QT_MIN_VERSION "5.4.0")

## Find Qt5
find_package(Qt5 ${QT_MIN_VERSION} CONFIG REQUIRED COMPONENTS
             Core
	         Widgets	
	         Quick)

add_subdirectory(src)

message(Include Dirs:\n${ROBOY_INCLUDE_DIRS})
message(Headers Found:\n${ROBOY_HEADERS})
message(Sources Found:\n${ROBOY_SOURCES})

## Dummy target to add non-source files to Qt-Project Viewer
file(GLOB ETC ${CMAKE_CURRENT_SOURCE_DIR}/etc/*)
file(GLOB DB  ${CMAKE_CURRENT_SOURCE_DIR}/database/*)
file(GLOB DB  ${CMAKE_CURRENT_SOURCE_DIR}/msg/*)
add_custom_target(dummy_target SOURCES ${ROBOY_HEADERS} ${ETC} ${DB})

## Generate Qt-Files (Mocs for Headers)
qt5_wrap_cpp(MOC_HDRS ${ROBOY_HEADERS})

## Adding Material resources
qt5_add_resources(MOC_RESOURCES ${ROBOY_RESOURCES}
  								$ENV{MATERIAL_DIR}/modules/FontAwesome.qrc
  								$ENV{MATERIAL_DIR}/modules/FontRoboto.qrc
  								$ENV{MATERIAL_DIR}/modules/Icons.qrc
  								$ENV{MATERIAL_DIR}/modules/Material.qrc
 						 		$ENV{MATERIAL_DIR}/modules/MaterialQtQuick.qrc)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

## Build Target
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../common_utilities/include
        ${ROBOY_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${QT_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(roboy_control_node ${ROBOY_SOURCES} ${MOC_HDRS} ${MOC_RESOURCES})
qt5_use_modules(roboy_control_node Core Widgets Quick)
target_link_libraries(roboy_control_node ${catkin_LIBRARIES})
add_dependencies(roboy_control_node common_utilities_generate_messages_cpp)
