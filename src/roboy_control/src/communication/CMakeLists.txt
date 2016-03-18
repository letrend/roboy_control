set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/IControllerCommunication.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IMasterCommunication.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ROSControllerCommunication.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ROSMasterCommunication.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ROSControllerCommunication.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ROSMasterCommunication.cpp
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)

