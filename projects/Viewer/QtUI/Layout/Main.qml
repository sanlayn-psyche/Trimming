import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ViewerModule

ApplicationWindow {
    visible: true
    width: 1000
    height: 800
    title: "Trimming Viewer"

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Toolbar
        RowLayout {
            Layout.fillWidth: true
            Layout.margins: 5
            spacing: 10

            CheckBox {
                text: "Show Cover"
                checked: viewer.showCover
                onCheckedChanged: viewer.showCover = checked
            }

            ComboBox {
                model: ["NURBS", "Bezier", "Mono", "None"]
                currentIndex: viewer.curveMode
                onCurrentIndexChanged: viewer.curveMode = currentIndex
            }

            CheckBox {
                text: "Show Split"
                checked: viewer.showKdNode
                onCheckedChanged: viewer.showKdNode = checked
            }

            CheckBox {
                text: "Show Sample"
                checked: viewer.showSample
                onCheckedChanged: viewer.showSample = checked
            }

            CheckBox {
                text: "Show Curve"
                checked: viewer.showCurve
                onCheckedChanged: viewer.showCurve = checked
            }
             
            CheckBox {
                text: "Show BBox"
                checked: viewer.showBDB1
                onCheckedChanged: viewer.showBDB1 = checked
            }

            Label {
                text: "Patch Id:"
            }

            SpinBox {
                from: 0
                to: 100000
                value: viewer.patchId
                editable: true
                onValueChanged: viewer.patchId = value
            }

            Item { Layout.fillWidth: true } // Spacer
        }

        // Viewer Area
        ViewerController {
            id: viewer
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            // Initial state can be set here or in C++ constructor
            // patchId: 0
        }
    }
}
