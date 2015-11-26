set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/IModelService.h
        ${CMAKE_CURRENT_SOURCE_DIR}/RoboyBehaviorXmlParser.h
        ${CMAKE_CURRENT_SOURCE_DIR}/XmlModelService.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/RoboyBehaviorXmlParser.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/XmlModelService.cpp
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
