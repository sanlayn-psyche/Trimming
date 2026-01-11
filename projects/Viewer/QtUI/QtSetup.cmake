
# QtSetup.cmake
# Configures Qt6 and QML for the Viewer project
list(APPEND CMAKE_PREFIX_PATH "E:/Qt/6.10.1/msvc2022_64")

find_package(Qt6 COMPONENTS Quick Core Gui REQUIRED)

qt_policy(SET QTP0001 NEW)

qt_standard_project_setup()

# Check against standard target name
set(VIEWER_TARGET TrimmingViewer)

set_property(TARGET ${VIEWER_TARGET} PROPERTY AUTOMOC ON)

qt_add_qml_module(${VIEWER_TARGET}
    URI ViewerModule
    VERSION 1.0
    QML_FILES
        "QtUI/Layout/Main.qml"
    SOURCES
        "QtUI/Controller/ViewerController.h"
        "QtUI/Controller/ViewerController.cpp"
)

target_link_libraries(${VIEWER_TARGET} PRIVATE
    Qt6::Quick
    Qt6::Core
    Qt6::Gui
)

target_include_directories(${VIEWER_TARGET} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/QtUI"
    "${CMAKE_CURRENT_SOURCE_DIR}/QtUI/Controller"
)

# Run windeployqt to copy DLLs and QML plugins
add_custom_command(TARGET ${VIEWER_TARGET} POST_BUILD
    COMMAND "E:/Qt/6.10.1/msvc2022_64/bin/windeployqt.exe"
            --qmldir "${CMAKE_CURRENT_SOURCE_DIR}/QtUI/Layout"
            --no-translations
            --compiler-runtime
            "$<TARGET_FILE:${VIEWER_TARGET}>"
    COMMENT "Running windeployqt for ${VIEWER_TARGET}..."
)
