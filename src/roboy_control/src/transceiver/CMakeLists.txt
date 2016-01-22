set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ITransceiverService.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ITransceiverServiceDelegate.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ROSMessageTransceiverService.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MyoController.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ROSMessageTransceiverService.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MyoController.cpp
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
