set(HDRS ${HDRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewController.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/MainWindow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/editorView/EditorView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/playerView/PlayerView.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/playerView/BehaviorListModel.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/recorderView/RecorderView.h
PARENT_SCOPE)

set(SRCS ${SRCS}
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/MainWindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/editorView/EditorView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/playerView/PlayerView.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/playerView/BehaviorListModel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mainWindow/recorderView/RecorderView.cpp
PARENT_SCOPE)

set(RESOURCES ${RESOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/qml.qrc
PARENT_SCOPE)

set(INCL ${INCL}
        ${CMAKE_CURRENT_SOURCE_DIR}
PARENT_SCOPE)
