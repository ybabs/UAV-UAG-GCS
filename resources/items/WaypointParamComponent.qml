import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Popup{
    id: singleModePopupMenu
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    height: 230
    visible: false
    focus:true
    signal okButtonClicked
    property var waypoint


    function addWaypoint(coordinate)
    {
        singleModePopupMenu.waypoint = coordinate
    }


    ColumnLayout{

        TextField{
            id:altitudeText
            Behavior on opacity {NumberAnimation{}}
            visible: opacity ? true: false
            Layout.fillWidth: true
            placeholderText: "Enter Altitude"
            validator: IntValidator{bottom: 1; top: 100}
            onEditingFinished: planner.altitude = altitudeText.text
        }

        CheckBox {
             id:landCheckBox
             text:"Land and Record"
             onClicked:
             {
                 if(checked ){
                     planner.sampleFlag = 1
                 }

                 else{
                     planner.sampleFlag = 0
                 }
             }
         }

        TextField{
             id:samplingText
             validator: IntValidator{
                 bottom: 1
                 top: 800
             }
             Layout.fillWidth:true
             placeholderText: "Enter Sampling Time"
             focus:true
             onEditingFinished: planner.samplingTime = samplingText.text
            inputMethodHints: Qt.ImhDigitsOnly
         }


        GroupBox{
            RowLayout{
                Button{
                    id:okPopupButton
                    text: "OK"
                    onClicked: {

                        onWaypointClickedSignal:{
                            planner.addWaypoint(waypoint.center.latitude, waypoint.center.longitude,
                            planner.altitude, planner.sampleFlag,
                            planner.samplingTime)

                        }

                        okButtonClicked()
                        missionPopup.close();
                    }
                }

                Button{
                    id:cancelPopupButton
                    text:"Cancel"
                    onClicked: {
                        missionPopup.close()
                        //TODO //Clear Waypoints
                    }
                }
            }
        }

    }



}
