import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ViewerModule

ApplicationWindow {
    visible: true
    width: 800
    height: 600
    title: "Trimming Viewer (Qt6)"

    ViewerController {
        id: controller
    }

    menuBar: MenuBar {
        Menu {
            title: "File"
            MenuItem {
                text: "Open..."
                onTriggered: console.log("Open triggered") 
            }
            MenuItem {
                text: "Exit"
                onTriggered: Qt.quit()
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        Text {
            text: "Status: " + controller.statusMessage
            font.pixelSize: 16
            Layout.alignment: Qt.AlignHCenter
        }

        Button {
            text: "Perform Trimming"
            Layout.alignment: Qt.AlignHCenter
            onClicked: controller.performTrimming()
        }

        Item {
            // Placeholder for rendering area
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            Rectangle {
                anchors.fill: parent
                color: "lightgray"
                border.color: "gray"
                
                Text {
                    anchors.centerIn: parent
                    text: "Render Area Placeholder"
                }
            }
        }
    }
}
