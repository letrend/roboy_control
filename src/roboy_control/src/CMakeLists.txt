set(HDRS
        ${CMAKE_CURRENT_SOURCE_DIR}/DataTypes.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LogDefines.h
)
set(SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/Main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DataTypes.cpp)
set(UI_FILES)
set(INCL ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(model)
add_subdirectory(view)
add_subdirectory(transceiver)
add_subdirectory(controller)
add_subdirectory(interfaces)

message(Include Directories:)
foreach(include_dir ${INCL})
        message(\t${include_dir})
endforeach()

set(ROBOY_SOURCES ${SRCS} PARENT_SCOPE)
set(ROBOY_HEADERS ${HDRS} PARENT_SCOPE)
set(ROBOY_INCLUDE_DIRS ${INCL} PARENT_SCOPE)
set(ROBOY_RESOURCES ${RESOURCES} PARENT_SCOPE)
