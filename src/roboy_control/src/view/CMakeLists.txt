set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/RecorderView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/EditorView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/RoboyBehaviorModel.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RecorderView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EditorView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RoboyBehaviorModel.cpp
PARENT_SCOPE)

set(UI_FILES ${UI_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/RecorderView.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/EditorView.ui
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
