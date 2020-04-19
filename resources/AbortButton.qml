import QtQuick 2.14
import QtQuick.Controls 2.14


Button{
        id:abortButton
        anchors {
            bottom: parent.bottom
            right: parent.right
            bottomMargin: 5
            rightMargin: 5
            
        }
        background: Rectangle{
             color: "red"
        }
        text: qsTr("Abort")
        opacity: 1

        onClicked:
        {
            planner.abortMission()
            console.log("abort clicked")
        }
    }

 