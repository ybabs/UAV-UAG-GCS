import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtQuick.Controls.Material 2.14

    RowLayout{

        property bool mav1checked
        property bool mav2checked
        property bool mav3checked
        property bool mav4checked

        CheckBox{
            id: mav1checkBox
            text: "MAV 1"
            font.bold:false
            onClicked: {
                if(checked)
                {
                    mav1checked = true
                   // planner.getMavId = 1
                }
                else
                {
                    mav1checked=false
                }
            }

        }

        CheckBox{
            id: mav2checkBox
            text: "MAV 2"
            onClicked: {
                if(checked)
                {
                    mav2checked = true
                    //planner.getMavId = 2
                }

                else 
                {
                    mav2checked = false
                }
            }

        }

        CheckBox{
            id: mav3checkBox
            text: "MAV 3"
            onClicked: {
                if(checked)
                {
                    mav3checked = true
                    //planner.getMavId = 3
                }
                else 
                {
                    mav3checked = false
                }
            }

        }

        CheckBox{
             id: mav4checkBox
             text: "MAV 4"
             onClicked: {
                 if(checked)
                 {
                     mav4checked = true
                    //planner.getMavId = 4
                 }

                 else
                 {
                     mav4checked = false
                 }
             }
        }

        Button{
            id: setMavControlButton
            highlighted:true
            text: "Upload"
            onClicked: {

                if(mav1checked)
                {
                    planner.getMavId = 1
                    //  planner.armMav()
                }
                if(mav2checked)
                {
                    planner.getMavId = 2
                }
                if(mav3checked)
                {
                    planner.getMavId = 3
                }
                if(mav4checked)
                {
                    planner.getMavId = 4
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
                planner.getMavId = 255;
               
                 planner.uploadWaypoints()

                // set Logic here



            }

        }

    }

