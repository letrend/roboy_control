set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/MainWindow.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/MainWindow.cpp
PARENT_SCOPE)

set(RESOURCES ${RESOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/qml.qrc
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
