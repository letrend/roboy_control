set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow/MainWindow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/PlayerView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/BehaviorListModel.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/BehaviorQueueModel.h
        ${CMAKE_CURRENT_SOURCE_DIR}/RecorderView/RecorderView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/EditorView/EditorView.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow/MainWindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/PlayerView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/BehaviorListModel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/BehaviorQueueModel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RecorderView/RecorderView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EditorView/EditorView.cpp
PARENT_SCOPE)

set(UI_FILES ${UI_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/MainWindow/MainWindow.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/PlayerView/PlayerView.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/RecorderView/RecorderView.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/EditorView/EditorView.ui
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
