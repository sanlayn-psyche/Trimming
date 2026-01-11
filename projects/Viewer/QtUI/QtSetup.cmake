
# QtSetup.cmake
# Configures Qt6 and QML for the Viewer project
# ==============================================================================
# USER CONFIGURATION REQUIRED
# Set this path to your Qt6 installation directory (the kit specific folder)
# Example Windows: "C:/Qt/6.8.0/msvc2019_64"
# Example MacOS:   "/Users/username/Qt/6.8.0/macos"
# ==============================================================================
set(QT_INSTALL_DIR "E:/Qt/6.10.1/msvc2022_64")

if(NOT EXISTS "${QT_INSTALL_DIR}")
    message(FATAL_ERROR "Qt installation not found at: ${QT_INSTALL_DIR}\nPlease edit projects/Viewer/QtUI/QtSetup.cmake and set QT_INSTALL_DIR to your Qt path.")
endif()

list(APPEND CMAKE_PREFIX_PATH "${QT_INSTALL_DIR}")
set(QT_BIN_DIR "${QT_INSTALL_DIR}/bin")

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

# Override RootPath to use CMAKE_CURRENT_SOURCE_DIR for portability
target_compile_definitions(${VIEWER_TARGET} PRIVATE RootPath="${CMAKE_CURRENT_SOURCE_DIR}")

# Deploy logic
if(WIN32)
    add_custom_command(TARGET ${VIEWER_TARGET} POST_BUILD
        COMMAND "${QT_BIN_DIR}/windeployqt.exe"
                --qmldir "${CMAKE_CURRENT_SOURCE_DIR}/QtUI/Layout"
                --no-translations
                --compiler-runtime
                "$<TARGET_FILE:${VIEWER_TARGET}>"
        COMMENT "Running windeployqt..."
    )
elseif(APPLE)
    add_custom_command(TARGET ${VIEWER_TARGET} POST_BUILD
        COMMAND "${QT_BIN_DIR}/macdeployqt"
                "$<TARGET_BUNDLE_DIR:${VIEWER_TARGET}>"
                -qmldir="${CMAKE_CURRENT_SOURCE_DIR}/QtUI/Layout"
                # -verbose=1
        COMMENT "Running macdeployqt..."
    )
endif()
