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
                 planner.getHoverFlag = 1
                 planner.getRthFlag = 0
                 planner.getLandFlag = 0
             }
        }
    RadioButton {
            id:rthRadioButton
            text: qsTr("RTH")
            onClicked:
             {
                 planner.getHoverFlag = 0
                 planner.getRthFlag = 1
                 planner.getLandFlag = 0
             }
        }
    RadioButton {
            id: landRadioButton
            text: qsTr("Land")
            onClicked:
             {
                 planner.getHoverFlag = 0
                 planner.getRthFlag = 0
                 planner.getLandFlag = 1
             }
        }
}
