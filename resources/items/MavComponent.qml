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

        MavCheckBox{
            text: "MAV 1"
            changeOnClick: true
            checked: mav1checked
            onClicked: {
                mav1checked = !mav1checked;
                if(mav1checked)
                {
                    planner.mavId = 1;
                }
                else if (!mav1checked)
                {
                    planner.mavId = 10;
                }
            }
        }

        MavCheckBox{
            text: "MAV 2"
            changeOnClick: true
            checked: mav2checked
            onClicked: {
                mav2checked = !mav2checked;
                if(mav2checked)
                {
                    planner.mavId = 2;
                }
                else if(!mav2checked)
                {
                    planner.mavId = 20;
                }
            }
        }

        MavCheckBox{
            text: "MAV 3"
            changeOnClick: true
            checked: mav3checked
            onClicked: {
                mav3checked = !mav3checked;
                if(mav3checked)
                {
                    planner.mavId = 3;
                }
                else if(!mav3checked)
                {
                    planner.mavId = 30;
                }
            }
        }

        MavCheckBox{
            text: "MAV 4"
            changeOnClick: true
            checked: mav4checked
            onClicked: {
                mav4checked = !mav4checked;
                if(mav4checked)
                {
                    planner.mavId = 4;
                }
                else if (!mav4checked)
                {
                    planner.mavId = 40;
                }
            }
        }


        Button{
            id: setMavControlButton
            highlighted:true
            text: "Upload"
            onClicked: {

                // if(mav1checked.status == checked)
                // {
                //     planner.mavId = 1
                //     //  planner.armMav()
                // }
                // if(mav2checked.status == checked)
                // {
                //     planner.mavId = 2
                // }
                // if(mav3checked.status == checked)
                // {
                //     planner.mavId = 3
                // }
                // if(mav4checked.status == checked)
                // {
                //     planner.mavId = 4
                // }
                
                planner.uploadWaypoints()
            }

        }
    }
