set(HDRS ${HDRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/RoboyControlConfiguration.h
    ${CMAKE_CURRENT_SOURCE_DIR}/RoboyController.h
PARENT_SCOPE)

set(SRCS ${SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/RoboyControlConfiguration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RoboyController.cpp
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
