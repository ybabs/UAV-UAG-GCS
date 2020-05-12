import QtQuick 2.14

Rectangle {
    id: root
    width: 40
    height: 40
    color: "#ffffff"
    border.color: Qt.darker(color, 1.2)
    property alias text: label.text
    property color fontColor: '#1f1f1f'
    Text {
        id: label
        anchors.centerIn: parent
        font.pixelSize: 5
        color: root.fontColor
    }
}
