
# QtSetup.cmake
# Configures Qt6 and QML for the Viewer project

find_package(Qt6 COMPONENTS Quick Core Gui REQUIRED)

qt_standard_project_setup()

# Check against standard target name
set(VIEWER_TARGET TrimmingViewer)

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
