import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14


RowLayout {
    id:missionConfigLayout

    RadioButton {
            id:noActionRadioButton
            checked: false
             text: qsTr("Hover")
             onClicked:
             {
                 planner.hoverFlag = 1
                 planner.rthFlag = 0
                 planner.landFlag = 0
             }
        }
    RadioButton {
            id:rthRadioButton
            text: qsTr("RTH")
            onClicked:
             {
                 planner.hoverFlag = 0
                 planner.rthFlag = 1
                 planner.landFlag = 0
             }
        }
    RadioButton {
            id: landRadioButton
            text: qsTr("Land")
            onClicked:
             {
                 planner.hoverFlag = 0
                 planner.rthFlag = 0
                 planner.landFlag = 1
             }
        }
}
