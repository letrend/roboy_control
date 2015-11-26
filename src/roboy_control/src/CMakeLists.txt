set(HDRS ${CMAKE_CURRENT_SOURCE_DIR}/DataTypes.h)
set(SRCS ${CMAKE_CURRENT_SOURCE_DIR}/RoboyControl.cpp)
set(UI_FILES)
set(INCL ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(model)
add_subdirectory(view)
add_subdirectory(controller)

message(Include Directories:)
foreach(include_dir ${INCL})
        message(\t${include_dir})
endforeach()

set(ROBOY_SOURCES ${SRCS} PARENT_SCOPE)
set(ROBOY_HEADERS ${HDRS} PARENT_SCOPE)
set(ROBOY_UI_FILES ${UI_FILES} PARENT_SCOPE)
set(ROBY_INCLUDE_DIRS ${INCL} PARENT_SCOPE)
