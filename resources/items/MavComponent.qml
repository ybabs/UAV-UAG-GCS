import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtQuick.Controls.Material 2.14

    RowLayout{

        property bool mav1checked
        property bool mav2checked
        property bool mav3checked
        property bool mav4checked
        property bool swarmCheckState
        signal swarmModeChecked(bool value)

        CheckBox{
            id:swarmCheckBox
            text: "Swarm Mode"
            onClicked:{
                if(checked)
                {
                    swarmCheckState = true
                }

                else
                {
                    swarmCheckState = false
                }

                swarmModeChecked(swarmCheckState)
            }
        }

        CheckBox{
            id: mav1checkBox
            text: "MAV 1"
            font.bold:false
            onClicked: planner.mavId = checked
            Component.onCompleted: checked = planner.mavId
            Connections{
                target: planner
                onStatusChanged: mav1checkBox.checked = planner.mavId
            }

        }

        CheckBox{
            id: mav2checkBox
            text: "MAV 2"
            onClicked: planner.mavId = checked
            Component.onCompleted: checked = planner.mavId
            Connections{
                target: planner
                onStatusChanged: mav2checkBox.checked = planner.mavId
            }

        }

        CheckBox{
            id: mav3checkBox
            text: "MAV 3"
            onClicked: planner.mavId = checked
            Component.onCompleted: checked = planner.mavId
            Connections{
                target: planner
                onStatusChanged: mav3checkBox.checked = planner.mavId
            }

        }

        CheckBox{
             id: mav4checkBox
             text: "MAV 4"
             onClicked: {
                 planner.mavId = 4;
                 checked = Qt.binding(function() {
                     return planner.mavId;
                 })
             }


        }

        Button{
            id: setMavControlButton
            highlighted:true
            text: "Upload"
            onClicked: {

                if(mav1checked.status == checked)
                {
                    planner.mavId = 1
                    //  planner.armMav()
                }
                if(mav2checked.status == checked)
                {
                    planner.mavId = 2
                }
                if(mav3checked.status == checked)
                {
                    planner.mavId = 3
                }
                if(mav4checked.status == checked)
                {
                    planner.mavId = 4
                }
                
                planner.uploadWaypoints()
            }

        }

        Button{
            id:setAllControlButton
            text:"Control All"
            onClicked: {

                mav1checkBox.checked = true;
                mav2checkBox.checked = true;
                mav3checkBox.checked = true;
                mav4checkBox.checked = true;
                planner.mavId = 255;
               
                 planner.uploadWaypoints()

                // set Logic here



            }

        }

    }

