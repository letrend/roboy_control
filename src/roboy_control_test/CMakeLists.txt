cmake_minimum_required(VERSION 2.8.3)
project(roboy_control_test)

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

catkin_package(
   CATKIN_DEPENDS message_runtime
)

## Build Target
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../common_utilities/include
        ${catkin_INCLUDE_DIRS}
        ${QT_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(roboy_control_test_node src/Main.cpp src/TestNode.cpp)
target_link_libraries(roboy_control_test_node ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(roboy_control_test_node common_utilities_generate_messages_cpp)

add_executable(controller_stub src/ControllerStub.cpp)
target_link_libraries(controller_stub ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(controller_stub  common_utilities_generate_messages_cpp)
